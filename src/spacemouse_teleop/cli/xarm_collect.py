"""Three-camera collection composed with the existing SpaceMouse teleop."""
from __future__ import annotations
import argparse
import hashlib
from dataclasses import asdict
from importlib.metadata import version, PackageNotFoundError
import json
import math
import multiprocessing as mp
from pathlib import Path
import signal
import subprocess
import sys
import time

from spacemouse_teleop.backends.xarm6_camera.config import CollectionConfig, DEFAULT_CONFIG
from spacemouse_teleop.backends.xarm6_camera.backend import XArmCameraBackend
from spacemouse_teleop.backends.xarm6_camera.coordinator import CollectionCoordinator
from spacemouse_teleop.backends.xarm6_camera.preview import Preview
from spacemouse_teleop.backends.xarm6_camera.state import StatePoller
from spacemouse_teleop.backends.xarm_sdk.input import LatestInput
from spacemouse_teleop.backends.xarm_sdk.transport import SDKChannel, ReportReceiver
from spacemouse_teleop.cli.xarm_teleop import run_loop
from spacemouse_teleop.recording.collection import CollectionRecorder
from spacemouse_teleop.sensors.realsense import CameraGroup, MockCameraGroup

class MockReadOnlyChannel:
    def __init__(self, ip, timeout, writable=False):
        if writable:
            raise ValueError('Mock acquisition has no motion commands')
    def query(self, name):
        t = time.monotonic()
        if name == 'get_joint_pos':
            return [math.sin(t)*.02, -.8, -.5, 0., 1.5, 0., 0.]
        if name == 'get_tcp_pose':
            return [300.+math.sin(t)*5., 0., 200., math.pi, 0., 0.]
        raise ValueError(name)
    def gripper_status(self):
        return {'position_pulse': 500., 'controller_target_pulse':500., 'status_code':0,
                'motion_state':'stopped','error_code':0,'received_at':time.monotonic()}
    def close(self):
        pass

def provenance(mode):
    packages = {}
    for name in ('spacemouse-teleop','pyrealsense2','numpy','opencv-python','xarm-python-sdk','pyspacemouse'):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    repo = Path(__file__).resolve().parents[3]
    try:
        revision = subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True,stderr=subprocess.DEVNULL,timeout=2).strip()
        dirty = bool(subprocess.check_output(['git','status','--porcelain'],cwd=repo,text=True,timeout=2).strip())
    except (OSError,subprocess.SubprocessError):
        revision, dirty = None, None
    return {'capture_mode':mode,'packages':packages,'git_revision':revision,'git_dirty':dirty,
            'device_measurement_refresh_verified':False,
            'source_sha256':{str(path.relative_to(repo)):hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in sorted((repo/'src'/'spacemouse_teleop').rglob('*.py'))},
            'joints_order':['joint1','joint2','joint3','joint4','joint5','joint6'],
            'tcp_frame':'base','tcp_rpy_convention':'R = Rz(yaw) Ry(pitch) Rx(roll)',
            'gripper_units':'SDK raw pulse; not calibrated opening or force'}

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(DEFAULT_CONFIG))
    parser.add_argument('--ip')
    parser.add_argument('--task',required=True)
    parser.add_argument('--output',help='Override collection output directory')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run',action='store_true',help='Synthetic cameras, robot and input; no hardware connections')
    mode.add_argument('--observe-only',action='store_true',help='RGB + read-only robot queries; no HID, enabling, homing or motion')
    parser.add_argument('--headless',action='store_true',help='No GUI; only dry-run or observe-only')
    parser.add_argument('--auto-record',action='store_true',help='Automatically save one test episode; only dry-run or observe-only')
    parser.add_argument('--duration',type=float,default=0.,help='0 means unlimited; test modes can set a finite duration')
    return parser

def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    if (args.headless or args.auto_record) and not (args.dry_run or args.observe_only):
        parser.error('--headless/--auto-record are test modes; live teleop uses explicit preview controls')
    if args.headless and (not args.auto_record or args.duration <= 0):
        parser.error('headless tests require --auto-record and a finite positive duration')
    recorder = cameras = poller = source = backend = preview = reports = channel = None
    coordinator = events = None
    result, reason = 0, 'completed'
    old_term = None
    try:
        config = CollectionConfig.load(args.config,ip=args.ip)
        data, robot = config.data, config.robot
        if not args.dry_run and any(not c.get('serial') for c in data['cameras']):
            raise ValueError('Bind all three camera serials before opening live cameras')
        if not (args.dry_run or args.observe_only):
            config.require_live()
        mode = 'mock' if args.dry_run else ('read_only' if args.observe_only else 'teleop')
        recorder = CollectionRecorder(args.output or data['output_root'],args.task,
                    {'collection':data,'robot':asdict(robot)},sample_hz=data['sample_hz'],
                    quality=data['quality'],min_free_bytes=data['min_free_bytes'],
                    queue_size=data['queue_size'],metadata=provenance(mode),
                    visual_storage=data['visual_storage'])
        recorder.preflight()
        if not (args.dry_run or args.observe_only):
            backend = XArmCameraBackend(robot,recorder)
            initial = backend.connect()
            recorder.metadata.update(robot_identity=backend.identity,initial_robot_feedback=initial.to_dict())
        elif args.observe_only:
            channel = SDKChannel(robot.robot['ip'],robot.robot['socket_timeout_s'],writable=False)
            reports = ReportReceiver(robot.robot['ip'],robot.robot['socket_timeout_s'])
            reports.start()
            initial = reports.wait_first(2.)
            recorder.metadata.update(robot_identity=channel.identity(),initial_robot_feedback=initial.to_dict())
        recorder.open()
        print(f'Collection session: {recorder.session_path}',flush=True)
        print(f'{mode}: target {data["sample_hz"]:g} observations/s; quality={data["quality"]}',flush=True)
        camera_type = MockCameraGroup if args.dry_run else CameraGroup
        camera_specs = [{**c,'serial':f'mock-{c["role"]}'} for c in data['cameras']] if args.dry_run else data['cameras']
        cameras = camera_type(camera_specs,recorder.publish_frame,
                              lambda error:recorder.mark_fault(str(error)))
        cameras.start()
        poller = StatePoller(robot.robot['ip'],recorder.publish_state,
                    lambda error:recorder.mark_fault(str(error)),hz=data['robot_poll_hz'],
                    timeout=robot.robot['socket_timeout_s'],
                    channel_factory=MockReadOnlyChannel if args.dry_run else SDKChannel,
                    gripper_hz=20. if args.dry_run or args.observe_only else 0.)
        poller.start()
        events = mp.get_context('spawn').Queue(maxsize=32)
        coordinator = CollectionCoordinator(recorder,events,auto_record=args.auto_record)
        if not args.headless:
            preview = Preview(cameras.latest_frames,coordinator.status,events,on_fault=recorder.mark_fault)
            coordinator.exit_event = preview.quit_event
            preview.start()
        old_term = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM,lambda signum,frame: (_ for _ in ()).throw(KeyboardInterrupt()))
        deadline = time.monotonic()+10.
        while len(cameras.latest_frames()) != 3:
            coordinator.check_exit()
            errors = cameras.diagnostics()['errors']
            if errors or time.monotonic() > deadline:
                raise RuntimeError(f'Cameras not ready: {errors}')
            time.sleep(.02)
        if args.observe_only:
            # A read-only observer has no reset, stop, activate or write methods.
            started, report_sequence = time.monotonic(), None
            coordinator.ready_phase = 'read_only'
            coordinator.set_phase('read_only')
            while not args.duration or time.monotonic()-started < args.duration:
                request = coordinator.poll(True)
                if request == 'prepare_next':
                    coordinator.message = 'Read-only mode: no robot reset is performed.'
                feedback = reports.snapshot()
                if feedback and feedback.sequence != report_sequence:
                    report_sequence = feedback.sequence
                    recorder.publish_state({'kind':'report','sequence':report_sequence,'valid':True,
                        'source':'readonly_rich_report','received_at':feedback.received_at,'values':feedback.to_dict()})
                if args.auto_record and recorder.status().get('state') == 'fault':
                    raise RuntimeError(recorder.status()['error'])
                time.sleep(.005)
        else:
            source = LatestInput(robot,backend='mock' if args.dry_run else robot.reader['backend'])
            source.start()
            while True:
                outcome = run_loop(robot,source,backend,recorder,args.duration,args.dry_run,coordinator)
                if outcome != 'prepare_next':
                    break
    except KeyboardInterrupt:
        reason = 'interrupted'
    except Exception as exc:
        result,reason = 1,str(exc)
        print(f'Collection failed: {exc}',file=sys.stderr,flush=True)
    finally:
        # The robot must stop before camera joins, encoder queues or file flushes.
        if backend:
            try:
                errors = backend.close()
                if errors:
                    result = 1
                    print('Motion cleanup: '+'; '.join(errors),file=sys.stderr)
            except Exception as exc:
                result = 1
                print(f'Motion cleanup failed: {exc}',file=sys.stderr)
        if recorder:
            recorder.stop_episode(reason,incomplete=result != 0 or reason == 'interrupted')
        for component in (source,poller,reports,channel,cameras,preview):
            if component:
                try:
                    component.close()
                except Exception as exc:
                    result = 1
                    print(f'Cleanup failed: {exc}',file=sys.stderr)
        if recorder:
            if args.auto_record:
                deadline = time.monotonic()+30.
                while recorder.status()['state'] == 'finalizing':
                    status = recorder.status()
                    if status.get('finalization_deadline') is not None:
                        deadline = status['finalization_deadline']+4
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(.02)
                if recorder.status()['complete']:
                    recorder.disposition('kept')
                else:
                    result = 1
            recorder.write({'event':'session_end','timestamp':time.monotonic(),'reason':reason,'status':result})
            recorder.close()
            status = recorder.status()
            if status.get('fatal_error'):
                result = 1
            print(json.dumps({'collection_result':status,'exit_code':result},allow_nan=False),flush=True)
        if events:
            events.cancel_join_thread()
            events.close()
        if old_term is not None:
            signal.signal(signal.SIGTERM,old_term)
    return result

if __name__ == '__main__':
    raise SystemExit(main())
