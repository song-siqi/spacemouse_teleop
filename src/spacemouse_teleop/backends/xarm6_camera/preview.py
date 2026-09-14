"""GUI lives in a separate process; it cannot call the robot SDK."""
from __future__ import annotations
import multiprocessing as mp
import os
import queue
import sys
import threading
import time

def _window(updates, events, quit_event):
    import cv2
    import numpy as np
    title = 'xArm RGB collection | Space record/end | K keep | X discard | N reset | Q exit'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    opened, content = False, None
    last_seen = {}
    keys = {32:'toggle', ord('k'):'keep', ord('x'):'discard', ord('n'):'prepare_next', ord('q'):'quit', 27:'quit'}
    try:
        while True:
            try:
                while True:
                    message = updates.get_nowait()
                    if message is None:
                        return
                    content = message
            except queue.Empty:
                pass
            if content:
                frames, status = content
                panels = []
                for role in ('wrist','fixed_1','fixed_2'):
                    frame = frames.get(role)
                    panel = np.zeros((360,480,3), dtype=np.uint8) if frame is None else cv2.resize(frame['rgb'][:,:,::-1],(480,360))
                    label = role + (' MISSING' if frame is None else ' '+frame['serial'])
                    cv2.putText(panel,label,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
                    panels.append(panel)
                canvas = np.concatenate(panels, axis=1)
                banner = np.zeros((120,1440,3),dtype=np.uint8)
                lines = [f"Motion: {status.get('motion_phase')} | Record: {status.get('state')} | Episode: {status.get('episode_id')} | Frames: {status.get('frame_counts',{})} | Robot: {status.get('state_counts',{}).get('robot',0)}",
                         'Space: start/end | K: keep | X: discard | N: open gripper + reset | Q: stop robot and exit',
                         str(status.get('error') or status.get('instruction') or status.get('message') or '')[:170]]
                for index,line in enumerate(lines):
                    color = (0,0,255) if index == 2 and status.get('error') else (255,255,255)
                    cv2.putText(banner,line,(8,25+index*32),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1)
                cv2.imshow(title,np.concatenate([canvas,banner],axis=0))
                opened = True
            key = cv2.waitKey(10) & 0xff
            event = keys.get(key)
            if event and event != 'quit':
                now = time.monotonic()
                previous = last_seen.get(key, -1e9)
                last_seen[key] = now
                if now - previous < .8:
                    event = None  # Ignore desktop key auto-repeat until a quiet release gap.
            if opened and cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE) < 1:
                event = 'quit'
            if event == 'quit':
                quit_event.set()
                return
            if event:
                try:
                    events.put_nowait(event)
                except queue.Full:
                    pass
    finally:
        cv2.destroyAllWindows()

class Preview:
    def __init__(self, frames, status, events, on_fault=None):
        self.frames, self.status, self.events = frames, status, events
        self.context = mp.get_context('spawn')
        self.updates = self.context.Queue(maxsize=2)
        self.quit_event = self.context.Event()
        self.on_fault = on_fault or (lambda reason: None)
        self.stop_event = threading.Event()
        self.process = self.thread = None

    def start(self):
        if os.name == 'posix' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise RuntimeError('Preview needs a desktop display; --headless is available for dry-run')
        self.process = self.context.Process(target=_window,args=(self.updates,self.events,self.quit_event),daemon=True)
        self.process.start()
        self.thread = threading.Thread(target=self._feed,name='collection-preview-feed',daemon=True)
        self.thread.start()

    def _feed(self):
        while not self.stop_event.is_set():
            if not self.process.is_alive():
                if self.process.exitcode == 0:
                    self.quit_event.set()
                else:
                    reason = f'Preview failed (exit {self.process.exitcode}); recording stopped. Ctrl-C exits teleop.'
                    self.on_fault(reason)
                    print(reason, file=sys.stderr, flush=True)
                return
            try:
                self.updates.put_nowait((self.frames(),self.status()))
            except queue.Full:
                pass
            self.stop_event.wait(.1)

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(.5)
        if self.process:
            try:
                self.updates.put_nowait(None)
            except queue.Full:
                pass
            self.process.join(1.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(1.)
        self.updates.cancel_join_thread()
        self.updates.close()
