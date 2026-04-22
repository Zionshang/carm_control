import sys
import termios
import tty
from typing import Any


class TerminalInputGuard:
    def __init__(self) -> None:
        self.fd: int | None = None
        self.attrs: list[Any] | None = None

    def __enter__(self) -> "TerminalInputGuard":
        if not sys.stdin.isatty():
            return self

        self.fd = sys.stdin.fileno()
        self.attrs = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if self.fd is None or self.attrs is None:
            return

        termios.tcflush(self.fd, termios.TCIFLUSH)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.attrs)
