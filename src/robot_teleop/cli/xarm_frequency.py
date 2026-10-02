"""Read-only C0 profiling and offline analysis; never enable or move a robot."""
from __future__ import annotations

import argparse
import json
import math
import queue
import statistics
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from robot_teleop.backends.real.xarm_sdk.config import DEFAULT_CONFIG, XArmConfig
from robot_teleop.backends.real.xarm_sdk.transport import ReportReceiver
from robot_teleop.backends.real.xarm_sdk.state import StatePoller


MEASUREMENT_FIELDS = {
    "joints": ("joints_rad",), "tcp": ("tcp_mm_rad",), "gripper": ("position_pulse",),
    "report": ("joints_rad", "tcp_mm_rad"),
}
# These are evidence filters, not a calibrated sensor noise model or freshness proof.
CHANGE_THRESHOLDS = {"joints": [1e-3]*6, "tcp": [.1]*3+[1e-3]*3,
                     "gripper": [1.], "report": [1e-3]*6+[.1]*3+[1e-3]*3}
MOTION_SPAN_THRESHOLDS = {"joints": [.01]*6, "tcp": [1.]*3+[.01]*3,
                          "gripper": [5.], "report": [.01]*6+[1.]*3+[.01]*3}


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    def percentile(p):
        i = (len(values) - 1) * p
        return values[int(i)] + (values[math.ceil(i)] - values[int(i)]) * (i % 1)
    return {"median": statistics.median(values), "p95": percentile(.95), "p99": percentile(.99), "max": max(values)}


def measurement_fingerprint(sample):
    """Measured positions only: timestamps, sequences, targets and status do not count."""
    values, result = sample.get("values", {}), []
    for field in MEASUREMENT_FIELDS.get(sample.get("kind"), ()):
        value = values.get(field)
        if value is None:
            return None
        items = value if isinstance(value, (list, tuple)) else [value]
        if any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) for v in items):
            return None
        result.extend(items)
    expected = len(CHANGE_THRESHOLDS.get(sample.get("kind"), ()))
    return tuple(result) if len(result) == expected and expected else None


def change_evidence(rows, kind):
    measured = [(row, measurement_fingerprint(row)) for row in rows]
    measured = [(row, value) for row, value in measured if value is not None]
    changed_times = [b[0]["received_at"] for a,b in zip(measured, measured[1:]) if a[1] != b[1]]
    threshold = CHANGE_THRESHOLDS.get(kind, ())
    material_times, anchor = [], measured[0][1] if measured else None
    for row, value in measured[1:]:
        if any(abs(b-a) >= limit for a,b,limit in zip(anchor, value, threshold)):
            material_times.append(row["received_at"])
            anchor = value
    spans = [max(values)-min(values) for values in zip(*(value for _,value in measured))] if measured else []
    return {
        "value_changes": len(changed_times),
        "measurement_fields": list(MEASUREMENT_FIELDS.get(kind, ())),
        "value_change_interval_s": distribution([b-a for a,b in zip(changed_times, changed_times[1:])]),
        "material_change_count": len(material_times),
        "material_change_interval_s": distribution([b-a for a,b in zip(material_times, material_times[1:])]),
        "material_change_thresholds": list(threshold), "measurement_component_spans": spans,
        "nontrivial_motion_span_observed": any(span >= limit for span,limit in zip(spans, MOTION_SPAN_THRESHOLDS.get(kind, ()))),
        "measurement_unavailable_count": len(rows)-len(measured),
        "change_evidence_note": "Host-observed changes, not device measurement timestamps. Stationary zero changes and small noise cannot establish or reject device update frequency.",
    }


def summarize(samples, started, ended, sample_hz=20.):
    groups = defaultdict(list)
    for sample in samples:
        if sample.get("kind") in MEASUREMENT_FIELDS:
            groups[sample['kind']].append(sample)
    summary = {}
    count = max(0, math.ceil((ended-started)*sample_hz-1e-6))
    # A predecessor just before the phase starts is usable for the first grid tick,
    # but is not counted as a response inside the measurement interval.
    for kind, all_rows in groups.items():
        all_rows = sorted(all_rows, key=lambda row: row['received_at'])
        rows = [r for r in all_rows if started <= r['received_at'] < ended]
        good = [r for r in rows if r.get('valid')]
        history = [r for r in all_rows if r.get('valid') and r['received_at'] < ended]
        timestamps = [r['received_at'] for r in good]
        intervals = [b-a for a,b in zip(timestamps, timestamps[1:])]
        rtts = [r['query_finished_at']-r['query_started_at'] for r in good if 'query_started_at' in r and 'query_finished_at' in r]
        ages, repeats, last, index = [], 0, None, -1
        for k in range(count):
            tick = started + k/sample_hz
            while index+1 < len(history) and history[index+1]['received_at'] <= tick:
                index += 1
            if index >= 0:
                ages.append(tick-history[index]['received_at'])
                repeats += int(last == history[index]['sequence'])
                last = history[index]['sequence']
        rate = (len(timestamps)-1)/(timestamps[-1]-timestamps[0]) if len(timestamps)>1 and timestamps[-1]>timestamps[0] else 0.
        summary[kind] = {"responses":len(rows), "valid_responses":len(good), "errors":len(rows)-len(good),
                         "response_hz":rate, "interval_s":distribution(intervals), "query_rtt_s":distribution(rtts),
                         **change_evidence(good, kind), "grid_ticks":count, "grid_age_s":distribution(ages),
                         "grid_repeated_source_fraction":repeats/len(ages) if ages else None,
                         "grid_missing_source_count":count-len(ages),
                         "grid_coverage_under_100ms":sum(a <= .1 for a in ages)/count if count else 0.,
                         "supports_response_rate":None if len(good)<2 else rate >= .95*sample_hz and len(good)==len(rows),
                         "device_update_conclusion":"insufficient_evidence"}
    return summary


def query_pair_summary(samples, started, ended):
    groups = defaultdict(dict)
    for sample in samples:
        if sample.get('valid') and started <= sample['received_at'] < ended:
            groups[sample['kind']][sample['sequence']] = sample
    pairs, skews, unmatched = [], [], 0
    for sequence in set(groups['joints']) | set(groups['tcp']):
        joints, tcp = groups['joints'].get(sequence), groups['tcp'].get(sequence)
        if not joints or not tcp or 'query_started_at' not in joints or 'query_finished_at' not in tcp:
            unmatched += 1
            continue
        if tcp['query_started_at'] < joints['query_finished_at']:
            unmatched += 1  # Do not pair unrelated simultaneous query sessions.
            continue
        pairs.append(tcp['query_finished_at']-joints['query_started_at'])
        skews.append(tcp['received_at']-joints['received_at'])
    gripper_spans = [r['query_finished_at']-r['query_started_at'] for r in groups['gripper'].values()
                     if 'query_started_at' in r and 'query_finished_at' in r]
    return {"matched_pairs":len(pairs), "unmatched_query_sequences":unmatched,
            "joint_tcp_pair_rtt_s":distribution(pairs), "joint_tcp_receive_skew_s":distribution(skews),
            "gripper_status_query_span_s":distribution(gripper_spans),
            "gripper_timing_note":"Span encloses sequential status/target/error/position queries; individual register acquisition times are unavailable and are not atomic."}


def _read_rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def summarize_control_log(path, started=None, ended=None):
    records = _read_rows(path)
    controls, sources, seen, session = [], [], set(), 0
    for row in records:
        if row.get('event') == 'session_start':
            session += 1
        raw, execution = row.get('raw', {}), row.get('execution', {})
        timestamp = raw.get('timestamp', execution.get('send_started_at'))
        if timestamp is None or not execution or 'speeds_mm_s_rad_s' not in execution:
            continue  # Exclude homing and dry-run rows.
        if (started is not None and timestamp < started) or (ended is not None and timestamp >= ended):
            continue
        controls.append((timestamp, execution))
        for kind, value in (('report', row.get('feedback')), ('gripper', row.get('gripper'))):
            if not value or 'received_at' not in value:
                continue
            identity = value.get('sequence') if kind == 'report' else value['received_at']
            key = session, kind, identity
            if key in seen:
                continue
            seen.add(key)
            sources.append({'kind':kind, 'sequence':len(seen), 'received_at':value['received_at'],
                            'values':value, 'valid':not value.get('error_code', 0)})
    controls.sort(key=lambda value: value[0])
    times = [item[0] for item in controls]
    sends = [value['send_started_at'] for _,value in controls if 'send_started_at' in value]
    acks = [value['ack_received_at']-value['send_started_at'] for _,value in controls
            if 'ack_received_at' in value and 'send_started_at' in value]
    begin = started if started is not None else (min(times) if times else None)
    end = ended if ended is not None else (max(times)+1e-6 if times else None)
    return {"path":str(path), "requested_interval":[started, ended], "control_rows":len(controls),
            "observed_interval":[min(times),max(times)] if times else None,
            "raw_sample_interval_s":distribution([b-a for a,b in zip(times,times[1:])]),
            "command_send_interval_s":distribution([b-a for a,b in zip(sends,sends[1:])]),
            "command_hz":(len(sends)-1)/(sends[-1]-sends[0]) if len(sends)>1 and sends[-1]>sends[0] else 0.,
            "command_ack_rtt_s":distribution(acks),
            "nonzero_arm_command_rows":sum(any(abs(v)>1e-9 for v in value['speeds_mm_s_rad_s']) for _,value in controls),
            "failed_command_rows":sum(value.get('return_code',0)!=0 for _,value in controls),
            "session_faults":sum(r.get('event')=='session_end' and r.get('status',0)!=0 for r in records),
            "deduplicated_sources":summarize(sources,begin,end) if begin is not None and end is not None else {},
            "note":"Feedback deduplicated by session/report sequence; gripper by session/received_at, never target_sequence. Compare phases only within the same host clock epoch."}


def compare_control_logs(control_log, baseline_control_log, started, ended):
    current = summarize_control_log(control_log, started, ended) if control_log else None
    baseline = summarize_control_log(baseline_control_log) if baseline_control_log else None
    delta = {}
    if current and baseline and current['control_rows'] and baseline['control_rows']:
        for name in ('command_send_interval_s','command_ack_rtt_s'):
            if current[name] and baseline[name]:
                delta[name] = {key:current[name][key]-baseline[name][key] for key in current[name]}
    return {'with_queries':current, 'baseline_without_queries':baseline, 'difference_s':delta,
            'comparison_available':bool(delta)}


def camera_overlap(samples, started, ended, diagnostic):
    camera_end = diagnostic['measured_at_monotonic']
    camera_start = camera_end-diagnostic['duration_s']
    lo, hi = max(started,camera_start), min(ended,camera_end)
    duration = max(0.,hi-lo)
    return {"camera_capture_interval":[camera_start,camera_end], "query_phase_interval":[started,ended],
            "overlap_interval":[lo,hi] if duration else None, "overlap_duration_s":duration,
            "acceptance_duration_met":duration >= 60.,
            "sources_during_overlap":summarize(samples,lo,hi) if duration else {},
            "query_pairs_during_overlap":query_pair_summary(samples,lo,hi) if duration else {},
            "note":"Only the host-monotonic interval intersection is concurrent. Camera per-frame history was not saved, so its full-window FPS cannot be recomputed over this intersection."}


def build_result(samples, started, ended, poll_hz, gripper_hz, failures=(), *, control_log=None,
                 baseline_control_log=None, rgb_diagnostic=None):
    sources = summarize(samples,started,ended)
    required = ('joints','tcp') + (('gripper',) if gripper_hz else ())
    supports = all(sources.get(k,{}).get('supports_response_rate') is True for k in required) if poll_hz else None
    result = {"analysis_version":2, "read_only":True, "started_at":started, "ended_at":ended,
              "duration_s":ended-started, "acceptance_duration_met":ended-started >= 60.,
              "poll_hz":poll_hz, "gripper_hz":gripper_hz, "sources":sources,
              "query_pairs":query_pair_summary(samples,started,ended), "failures":list(failures),
              "supports_response_rate":supports, "response_rate_target_hz":20.,
              "device_update_conclusion":"insufficient_evidence", "device_update_rate_hz":None,
              "outside_window_samples":sum(not started <= r['received_at'] < ended for r in samples),
              "note":"Response-rate support uses >=95% of the 20 Hz target with no response errors. It is not acceptance or proof of controller internal measurement rate; >=60 s and motion/control comparison are separate requirements."}
    if control_log or baseline_control_log:
        comparison = compare_control_logs(control_log,baseline_control_log,started,ended)
        result['control_comparison'] = comparison
        active = comparison['with_queries']
        has_motion = active and active['nonzero_arm_command_rows'] > 0
        if has_motion and any(source['nontrivial_motion_span_observed'] for key,source in sources.items() if key in ('joints','tcp')):
            result['device_update_conclusion'] = 'motion_correlated_position_changes_observed_internal_timing_unverified'
    if rgb_diagnostic:
        result['rgb_concurrency'] = camera_overlap(samples,started,ended,rgb_diagnostic)
    result['response_failures'] = bool(failures) or any(not sample.get('valid', False) for sample in samples)
    return result


def profile(ip, timeout, duration, poll_hz, gripper_hz, output, **analysis_options):
    samples, failures = [], []
    reports = poller = None
    inbox = queue.Queue(maxsize=10000)
    def publish(sample):
        try:
            inbox.put_nowait(sample)
        except queue.Full:
            failures.append({"reason":"diagnostic_queue_full"})
    started = time.monotonic()
    ended, sequence = started, None
    with Path(output).open('x',encoding='utf-8') as stream:
        def write(value):
            stream.write(json.dumps(value,allow_nan=False)+'\n')
        def drain():
            while True:
                try:
                    sample = inbox.get_nowait()
                except queue.Empty:
                    return
                samples.append(sample)
                write(sample)
        def report_sample():
            nonlocal sequence
            feedback = reports.snapshot()
            if feedback and feedback.sequence != sequence:
                sequence = feedback.sequence
                publish({"kind":"report", "sequence":sequence, "received_at":feedback.received_at,
                         "source":"existing_rich_report", "values":feedback.to_dict(), "valid":True})
        try:
            reports = ReportReceiver(ip,timeout)
            reports.start()
            initial = reports.wait_first(2.)
            if poll_hz:
                poller = StatePoller(ip,publish,failures.append,hz=poll_hz,timeout=timeout,gripper_hz=gripper_hz)
                poller.start()
            started = time.monotonic()
            deadline = started+duration
            write({"event":"profile_start","read_only":True,"timestamp":started,
                   "wall_time_utc":datetime.now(timezone.utc).isoformat(), "poll_hz":poll_hz,
                   "gripper_hz":gripper_hz,"initial_feedback":initial.to_dict()})
            while time.monotonic() < deadline:
                report_sample()
                drain()
                time.sleep(.002)
            ended = time.monotonic()
        except (RuntimeError,OSError,ValueError,KeyboardInterrupt) as exc:
            ended = time.monotonic()
            failures.append({'source':'profile','reason':str(exc) or type(exc).__name__})
        finally:
            # Producers must finish before the final drain/file close. In-flight
            # tail responses remain in raw evidence; the phase window excludes
            # responses received after ended_at from its rate statistics.
            if poller:
                try:
                    poller.close()
                except Exception as exc:
                    failures.append({'source':'state_shutdown','reason':str(exc)})
            if reports:
                try:
                    report_sample()
                except Exception as exc:
                    failures.append({'source':'report_shutdown','reason':str(exc)})
                try:
                    reports.close()
                except Exception as exc:
                    failures.append({'source':'report_shutdown','reason':str(exc)})
            drain()
            write({'event':'profile_end','timestamp':ended,'drained_at':time.monotonic(),'failures':failures})
    return build_result(samples,started,ended,poll_hz,gripper_hz,failures,**analysis_options)


def analyze_directory(path, **analysis_options):
    root = Path(path)
    previous = json.loads((root/'summary.json').read_text()) if (root/'summary.json').exists() else []
    results = []
    for index, filename in enumerate(sorted(root.glob('phase_*.jsonl'),key=lambda p:int(p.stem.split('_')[-1]))):
        rows = _read_rows(filename)
        header = next((row for row in rows if row.get('event')=='profile_start'),None)
        footer = next((row for row in reversed(rows) if row.get('event')=='profile_end'),None)
        if header is None:
            raise ValueError(f'Missing profile_start: {filename}')
        started = header['timestamp']
        if footer:
            ended, failures = footer['timestamp'], footer.get('failures',[])
        elif index < len(previous) and 'duration_s' in previous[index]:
            ended, failures = started+previous[index]['duration_s'], previous[index].get('failures',[])
        else:
            raise ValueError(f'No measured end time for {filename}; refusing to invent an interval')
        samples = [row for row in rows if row.get('kind') in MEASUREMENT_FIELDS]
        result = build_result(samples,started,ended,header['poll_hz'],header.get('gripper_hz',0.),failures,**analysis_options)
        result.update(raw_file=str(filename), historical_tail_drain_verified=footer is not None)
        results.append(result)
    if not results:
        raise ValueError(f'No phase logs in {root}')
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(DEFAULT_CONFIG))
    parser.add_argument('--ip')
    parser.add_argument('--duration',type=float,default=60.,help='Seconds per phase; acceptance requires >=60.')
    parser.add_argument('--rates',type=float,nargs='+',default=[0.,20.,30.])
    parser.add_argument('--gripper-hz',type=float,default=0.,help='Separate read-only polling; leave 0 during an existing live teleop worker.')
    parser.add_argument('--output',required=True)
    parser.add_argument('--analyze',help='Reanalyze this existing C0 directory offline; never connect to hardware.')
    parser.add_argument('--control-log',help='Existing live teleop JSONL, restricted to the C0 phase overlap.')
    parser.add_argument('--baseline-control-log',help='Existing teleop JSONL recorded without the additional queries.')
    parser.add_argument('--rgb-diagnostic',help='Existing camera diagnostic.json; calculate actual overlap only.')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration<=0 or any(not math.isfinite(r) or r<0 for r in args.rates) or not math.isfinite(args.gripper_hz) or args.gripper_hz<0:
        parser.error('Duration must be positive and frequencies nonnegative, all finite')
    try:
        output = Path(args.output)
        output.mkdir(parents=True,exist_ok=False)
        options = {'control_log':args.control_log,'baseline_control_log':args.baseline_control_log,
                   'rgb_diagnostic':json.loads(Path(args.rgb_diagnostic).read_text()) if args.rgb_diagnostic else None}
        if args.analyze:
            results = analyze_directory(args.analyze,**options)
            (output/'summary.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
        else:
            config = XArmConfig.load(args.config,ip=args.ip)
            results = []
            for index,hz in enumerate(args.rates):
                print(f'Read-only phase {index+1}: pairs={hz:g} Hz, duration={args.duration:g} s',flush=True)
                result = profile(config.robot['ip'],config.robot['socket_timeout_s'],args.duration,hz,
                                 args.gripper_hz if hz else 0.,output/f'phase_{index}.jsonl',**options)
                results.append(result)
                (output/'summary.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
                print(json.dumps(result,allow_nan=False),flush=True)
        if any(not result['acceptance_duration_met'] for result in results):
            print('Short diagnostic only: a phase/overlap shorter than 60 seconds is not an acceptance run.',flush=True)
        return 1 if any(result['response_failures'] for result in results) else 0
    except (RuntimeError,OSError,ValueError,KeyError) as exc:
        print(f'Read-only frequency diagnosis failed: {exc}',flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
