"""Terminal keyboard with short leases; no motion persists after key repeat stops."""
import select
import sys
import termios
import time
import tty
from robot_teleop.core import CartesianTwist, GripperCommand


class KeyboardInput:
    def __init__(self, stream=None, clock=time.monotonic):
        self.stream, self.clock = stream or sys.stdin, clock
        self.previous = None
        self.until = 0.
        self.command = (CartesianTwist(), GripperCommand())

    def open(self):
        if not self.stream.isatty():
            raise ValueError("Keyboard input requires an interactive terminal")
        self.previous = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream.fileno())

    def sample(self, elapsed):
        if select.select([self.stream], [], [], 0)[0]:
            key = self.stream.read(1)
            if key in ("q", "\x1b", "\x03"):
                raise KeyboardInterrupt
            linear = {"w": (.05,0,0), "s":(-.05,0,0), "a":(0,.05,0), "d":(0,-.05,0), "r":(0,0,.05), "f":(0,0,-.05)}.get(key,(0.,0.,0.))
            angular = {"j": (0,0,.3), "l":(0,0,-.3), "i":(0,.3,0), "k":(0,-.3,0), "u":(.3,0,0), "o":(-.3,0,0)}.get(key,(0.,0.,0.))
            self.command = CartesianTwist(linear,angular), GripperCommand({"[":"open", "]":"close"}.get(key,"hold"))
            self.until = self.clock()+.15
        return self.command if self.clock() < self.until else (CartesianTwist(),GripperCommand())

    def close(self):
        if self.previous is not None:
            termios.tcsetattr(self.stream,termios.TCSADRAIN,self.previous)
