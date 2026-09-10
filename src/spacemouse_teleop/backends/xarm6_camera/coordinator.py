"""Episode controls communicate with the robot loop only through lifecycle events."""
from __future__ import annotations
import queue
import time

class CollectionCoordinator:
    def __init__(self, recorder, events=None, auto_record=False):
        self.recorder = recorder
        self.events = events if events is not None else queue.Queue(maxsize=32)
        self.phase = 'preparing'
        self.auto_record = auto_record
        self.auto_started = False
        self.quit_requested = False
        self.message = ''
        self._deferred = []
        self._last_key = {}

    def set_phase(self, phase):
        self.phase = phase

    def _drain(self):
        items, self._deferred = self._deferred, []
        while True:
            try:
                items.append(self.events.get_nowait())
            except queue.Empty:
                break
        return items

    def check_exit(self):
        if getattr(self, 'exit_event', None) is not None and self.exit_event.is_set():
            self.quit_requested = True
        for event in self._drain():
            if event == 'quit':
                self.quit_requested = True
            elif self.phase != 'preparing':
                self._deferred.append(event)
        if self.quit_requested:
            raise KeyboardInterrupt

    def poll(self, running):
        self.phase = getattr(self, 'ready_phase', 'teleop') if running else 'waiting_neutral'
        self.check_exit()
        if self.auto_record and running and not self.auto_started:
            result = self.recorder.start_episode()
            self.auto_started = result is not None
        for event in self._drain():
            now = time.monotonic()
            if now - self._last_key.get(event, -1e9) < .35:
                continue
            self._last_key[event] = now
            status = self.recorder.status()
            if event == 'toggle':
                if not running:
                    self.message = 'Move then release SpaceMouse and both buttons; wait for TELEOP ACTIVE before Space.'
                elif status['state'] == 'recording':
                    self.recorder.stop_episode('user')
                elif running and status['state'] == 'idle':
                    self.recorder.start_episode()
                else:
                    self.message = 'Finish/dispose the current episode; recording also requires healthy sources.'
            elif event in ('keep', 'discard'):
                self.recorder.disposition('kept' if event == 'keep' else 'discarded')
            elif event == 'prepare_next':
                if running and status['state'] == 'idle':
                    self.phase = 'preparing'
                    self.auto_started = False
                    return 'prepare_next'
                self.message = 'End the episode and choose K (keep) or X (discard) before N.'
        return None

    def status(self):
        instructions = {
            'preparing':'Opening gripper + resetting arm. Recording disabled.',
            'waiting_neutral':'Move then release SpaceMouse and BOTH buttons. Wait for TELEOP ACTIVE before Space.',
            'teleop':'TELEOP ACTIVE. Space starts/ends recording; SpaceMouse still controls the robot.',
            'read_only':'READ ONLY. Robot is not controlled by this process.'}
        labels = {'preparing':'RESETTING','waiting_neutral':'WAITING FOR SPACEMOUSE',
                  'teleop':'TELEOP ACTIVE','read_only':'READ ONLY'}
        recording = self.recorder.status()
        archive = recording.get('archive', {})
        if recording['state'] == 'finalizing' and archive:
            instructions[self.phase] = (f"Video {archive.get('phase')}: {archive.get('role') or 'all cameras'} "
                f"{archive.get('frames',0)}/{archive.get('total_frames',recording.get('frame_count',0))}. "
                'Teleop continues; wait before K/X and N.')
        return {**recording, 'motion_phase': labels.get(self.phase,self.phase),
                'instruction':instructions.get(self.phase,''), 'message':self.message}
