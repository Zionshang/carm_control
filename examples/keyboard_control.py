#!/usr/bin/env python3
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from controller.tcp_carm import DEFAULT_TCP_OFFSET, TcpCarm
from utils.terminal_utils import TerminalInputGuard


CONTROL_DT = 0.02
POS_STEP = 0.002
ROT_STEP = 0.01
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class KeyboardInput:
    keyboard: Any
    stop_event: threading.Event = field(default_factory=threading.Event)
    keys: dict[object, bool] = field(default_factory=dict)
    listener: Any | None = None

    def __post_init__(self) -> None:
        key = self.keyboard.Key
        self.keys = {
            key.up: False,
            key.down: False,
            key.space: False,
            self.char_key("w"): False,
            self.char_key("s"): False,
            self.char_key("a"): False,
            self.char_key("d"): False,
            self.char_key("j"): False,
            self.char_key("l"): False,
            self.char_key("u"): False,
            self.char_key("o"): False,
            self.char_key("i"): False,
            self.char_key("k"): False,
        }

    def char_key(self, char: str) -> object:
        return self.keyboard.KeyCode.from_char(char)

    def norm_key(self, key: object) -> object:
        if isinstance(key, self.keyboard.KeyCode) and key.char:
            return self.char_key(key.char.lower())
        return key

    def on_press(self, key: object) -> bool | None:
        key = self.norm_key(key)
        if key == self.keyboard.Key.esc:
            self.request_stop()
            return False
        if key in self.keys:
            self.keys[key] = True
        return None

    def on_release(self, key: object) -> None:
        key = self.norm_key(key)
        if key in self.keys:
            self.keys[key] = False

    def axis(self, pos_key: object, neg_key: object) -> int:
        return int(self.keys[pos_key]) - int(self.keys[neg_key])

    def start(self) -> None:
        self.stop_event.clear()
        self.listener = self.keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

    def request_stop(self) -> None:
        print("\nEsc pressed, exiting keyboard control...")
        self.stop_event.set()

    def should_stop(self) -> bool:
        return self.stop_event.is_set()

    def wait(self, timeout: float) -> bool:
        return self.stop_event.wait(timeout)

    def stop(self) -> None:
        self.stop_event.set()
        if self.listener is not None:
            self.listener.stop()
            self.listener = None


def clamp_target(target: list[float]) -> None:
    target[3] = clamp(target[3], -math.pi, math.pi)
    target[4] = clamp(target[4], -math.pi / 2, math.pi / 2)
    target[5] = clamp(target[5], -math.pi, math.pi)


def update_target(target: list[float], keyboard_input: KeyboardInput) -> None:
    axis = keyboard_input.axis
    key = keyboard_input.keyboard.Key
    char_key = keyboard_input.char_key

    target[0] += axis(char_key("w"), char_key("s")) * POS_STEP
    target[1] += axis(char_key("a"), char_key("d")) * POS_STEP
    target[2] += axis(key.up, key.down) * POS_STEP
    target[3] += axis(char_key("u"), char_key("o")) * ROT_STEP
    target[4] += axis(char_key("i"), char_key("k")) * ROT_STEP
    target[5] += axis(char_key("j"), char_key("l")) * ROT_STEP
    clamp_target(target)


def reset_to_home(robot: TcpCarm, keyboard_input: KeyboardInput) -> list[float]:
    robot.move_joint(HOME_JOINTS, is_sync=True)
    time.sleep(0.5)  # 等待机械臂稳定
    keyboard_input.keys[keyboard_input.keyboard.Key.space] = False
    return robot.get_tcp_pose()


def print_help() -> None:
    print("Carm TCP keyboard teleop")
    print("w/s: x, a/d: y, up/down: z")
    print("u/o: roll, i/k: pitch, j/l: yaw")
    print("space: home, esc: quit")


def main() -> None:
    from pynput import keyboard

    keyboard_input = KeyboardInput(keyboard)
    robot = TcpCarm()

    try:
        robot.set_ready()
        robot.set_tcp_offset(DEFAULT_TCP_OFFSET)
        target = reset_to_home(robot, keyboard_input)
        robot.track_tcp_pose(target)

        with TerminalInputGuard():
            keyboard_input.start()
            print_help()

            while not keyboard_input.should_stop():
                if keyboard_input.keys[keyboard_input.keyboard.Key.space]:
                    target = reset_to_home(robot, keyboard_input)

                update_target(target, keyboard_input)
                robot.track_tcp_pose(target)
                print(
                    f"tcp target: {[round(x, 3) for x in target]}",
                    end="\r",
                    flush=True,
                )
                keyboard_input.wait(CONTROL_DT)
    except KeyboardInterrupt:
        keyboard_input.stop_event.set()
        print("")
    finally:
        keyboard_input.stop()
        robot.disconnect()


if __name__ == "__main__":
    main()
