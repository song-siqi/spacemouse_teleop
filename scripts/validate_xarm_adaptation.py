#!/usr/bin/env python3
"""Offline migration regression: read raw v3, replay sources, verify raw v4.

No robot, HID or camera connections. Generated episodes are test artifacts,
not newly collected demonstrations. The source session is never modified.
"""
from __future__ import annotations
import argparse
import hashlib
import heapq
import json
from pathlib import Path
import subprocess
import tempfile
import time
import numpy as np
from robot_teleop.backends.real.xarm_sdk.collection import XArmCollectionRecorder
from robot_teleop.recording.raw import check_episode


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def json_rows(path):
    with path.open() as f:
        for line in f:yield json.loads(line)


def stream_states(ep,kind):
    packet=(ep/'robot_reports.bin').open('rb') if kind=='robot' else None
    try:
        for row in json_rows(ep/f'{kind}_states.jsonl'):
            if packet:
                packet.seek(row['packet_offset']);row['packet']=packet.read(row['packet_bytes'])
            yield row['received_at'],kind,row
    finally:
        if packet:packet.close()


def stream_frames(ep,role):
    with tempfile.TemporaryFile() as errors:
        p=subprocess.Popen(['ffmpeg','-nostdin','-v','error','-threads','2','-i',str(ep/'videos'/f'{role}.mp4'),
            '-map','0:v:0','-an','-vsync','0','-pix_fmt','rgb24','-threads','2','-f','rawvideo','pipe:1'],
            stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=errors)
        try:
            for row in json_rows(ep/'frames'/f'{role}.jsonl'):
                size=row['width']*row['height']*3;data=p.stdout.read(size)
                if len(data)!=size:raise ValueError('Truncated decoded source frame')
                row={k:v for k,v in row.items() if k not in ('file','video_frame_index','video_pts_s')}
                row['rgb']=np.frombuffer(data,dtype=np.uint8).reshape((row['height'],row['width'],3))
                yield row['received_at'],role,row
            if p.stdout.read(1):raise ValueError('Unindexed source video frames')
            if p.wait(timeout=10):raise ValueError('Source video decode failed')
        finally:
            p.stdout.close()
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=2)
                except subprocess.TimeoutExpired:p.kill();p.wait()


def stream_control(session,start,end):
    for row in json_rows(session/'control.jsonl'):
        t=row.get('command',{}).get('timestamp',row.get('timestamp'))
        if t is not None and start<=t<=end:yield t,'commands',row


def replay(ep,output):
    session=ep.parent.parent
    meta=json.loads((ep/'meta.json').read_text());sm=json.loads((session/'session.json').read_text())
    if meta.get('format_version')!=3:raise ValueError('This regression harness requires raw v3')
    original={str(p):digest(p) for p in [*session.glob('*.json*'),*ep.rglob('*')] if p.is_file()}
    original_check=check_episode(ep)
    if meta['status']=='complete' and not original_check['valid']:raise ValueError(original_check)
    now=[meta['capture_requested_start_time']]
    rec=XArmCollectionRecorder(output,'replay_'+session.name+'_'+ep.name,meta['config'],cameras=meta['cameras'],
        pre_roll_s=meta['pre_roll_s'],post_roll_s=meta['post_roll_s'],min_free_bytes=0,
        clock=lambda:now[0],visual_storage=meta['visual_storage'],
        metadata={'capture_mode':'migration_replay','original_session':str(session.resolve()),
                  'original_episode':str(ep.resolve()),'original_status':meta['status'],
                  'original_disposition':meta['disposition'],
                  'clock_note':'Virtual replay of historical monotonic coordinates; not a live clock mapping'})
    rec.open()
    # The harness advances a controlled replay clock. No wall-time supervisor.
    rec._stop.set();rec.thread.join(2)
    rec.meta['clock_semantics']['host']='virtual historical coordinates for migration regression only'
    streams=[stream_states(ep,k) for k in ('robot','gripper','auxiliary')]
    # Runtime source id for legacy auxiliary feedback is "report".
    def auxiliary(stream):
        for t,k,row in stream:yield t,'report',row
    streams[2]=auxiliary(streams[2])
    streams += [stream_frames(ep,c['role']) for c in meta['cameras']]
    streams += [stream_control(session,meta['capture_requested_start_time'],meta['capture_requested_end_time'])]
    heap=[]
    for i,stream in enumerate(streams):
        value=next(stream,None)
        if value is not None:heapq.heappush(heap,(value[0],i,value))
    started=stopped=False
    try:
        while heap:
            t,i,(_,kind,row)=heapq.heappop(heap)
            if not started and t>meta['start_time']:
                now[0]=meta['start_time']
                if rec.start_episode() is None:raise ValueError('Replay sources not ready')
                rec.session.flush(wait_finalization=False);started=True
            if started and not stopped and t>meta['end_time']:
                now[0]=meta['end_time'];rec.stop_episode(meta['end_reason'],meta['status']!='complete')
                rec.session.flush(wait_finalization=False);stopped=True
            now[0]=t
            if kind=='commands':rec.write(row)
            elif kind in ('robot','gripper','report'):rec.publish_state(row)
            else:rec.publish_frame(row)
            # Test replay applies backpressure; real publishers remain nonblocking.
            if rec.session.queue.qsize()>32:rec.session.flush(wait_finalization=False)
            active=rec.session.active
            deadline=time.monotonic()+60
            while active and any(q.qsize()>=q.maxsize//2 for q in active.queues.values()):
                if active.errors or time.monotonic()>deadline:raise ValueError(active.errors or 'Replay writer stalled')
                time.sleep(.002)
            value=next(streams[i],None)
            if value is not None:heapq.heappush(heap,(value[0],i,value))
        if not started:raise ValueError('No operation interval in source')
        if not stopped:
            now[0]=meta['end_time'];rec.stop_episode(meta['end_reason'],meta['status']!='complete')
        rec.session.flush(wait_finalization=False)
        now[0]=meta['capture_requested_end_time']+1e-6
        rec.session.tick(rec._stamp());rec.session.flush()
        generated=rec.session.last
    finally:
        for stream in streams:
            stream.close()
        rec.close()
    report=check_episode(generated.path)
    expected_complete=meta['status']=='complete'
    if report['valid']!=expected_complete:raise ValueError(report)
    for key,value in meta['state_counts'].items():
        key='report' if key=='auxiliary' else key
        if generated.meta['counts'].get(key)!=value:raise ValueError(f'State count differs: {key}')
    for role,count in meta['frame_counts'].items():
        if generated.meta['counts'].get(role)!=count:raise ValueError(f'Frame count differs: {role}')
        source=list(json_rows(ep/'frames'/f'{role}.jsonl'))
        result=list(json_rows(generated.path/f'{role}.jsonl'))
        for old,new in zip(source,result):
            expected={k:v for k,v in old.items() if k not in ('file','video_frame_index','video_pts_s')}
            if expected!=new['values']:raise ValueError('Frame metadata differs')
    for kind in ('robot','gripper','auxiliary'):
        target='report' if kind=='auxiliary' else kind
        source=list(json_rows(ep/f'{kind}_states.jsonl'))
        result=list(json_rows(generated.path/f'{target}.jsonl'))
        if [r['values'] for r in result]!=source:raise ValueError(f'Native state differs: {kind}')
    if digest(ep/'robot_reports.bin')!=digest(generated.path/'robot.bin'):raise ValueError('Original packet bytes differ')
    for key in ('start_time','end_time','end_reason'):
        if generated.meta[key]!=meta[key]:raise ValueError(f'Operation boundary differs: {key}')
    if any(digest(Path(p))!=h for p,h in original.items()):raise ValueError('Source session changed')
    return dict(source=str(ep.resolve()),original_check=original_check,output=str(generated.path),
                replay_check=report,counts=generated.meta['counts'],
                source_unchanged=True,native_values_equal=True,frame_metadata_equal=True,
                original_packets_equal=True,operation_boundaries_equal=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episodes',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    results=[]
    for ep in args.episodes:
        result=replay(ep,args.output);results.append(result)
        (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('original_check',)},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
