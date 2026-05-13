from __future__ import annotations

import math
import queue
import threading
from typing import Any, Protocol

import lcm
from carm_waypoint import WaypointCommand

from controller.tcp_carm import TcpCarm
from controller.waypoint_store import WaypointStore

OP_TRACK_TCP = "TRACK_TCP"
OP_SAVE_CURRENT = "SAVE_CURRENT"
OP_GOTO = "GOTO"
OP_HOME = "HOME"
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
TCP_COMMAND_DIM = 6


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_tcp_target(target: list[float]) -> None:
    target[3] = clamp(target[3], -math.pi, math.pi)
    target[4] = clamp(target[4], -math.pi / 2, math.pi / 2)
    target[5] = clamp(target[5], -math.pi, math.pi)


class SupportsLcm(Protocol):
    def subscribe(self, channel: str, callback: Any) -> Any:
        ...

    def handle_timeout(self, timeout_ms: int) -> Any:
        ...


class LcmWaypointService:
    def __init__(
        self,
        robot: TcpCarm,
        store: WaypointStore,
        lcm_client: SupportsLcm,
        *,
        cmd_channel: str = "tcp_cmd",
        handle_timeout_ms: int = 100,
    ) -> None:
        self.robot = robot
        self.store = store
        self.lcm = lcm_client
        self.cmd_channel = cmd_channel
        self.handle_timeout_ms = handle_timeout_ms

        self._command_queue: queue.Queue[WaypointCommand] = queue.Queue()
        self._stop_event = threading.Event()
        self._command_thread: threading.Thread | None = None
        self._subscription: Any = None
        self._state_lock = threading.Lock()
        self._motion_thread: threading.Thread | None = None
        self._tcp_target: list[float] | None = None

    @classmethod
    def from_robot_config(
        cls,
        *,
        addr: str = "10.42.0.101",
        lcm_url: str = "udpm://239.255.76.67:7667?ttl=1",
        waypoint_file: str = "data/waypoints.json",
        cmd_channel: str = "tcp_cmd",
    ) -> "LcmWaypointService":
        robot = TcpCarm(addr=addr)
        store = WaypointStore(waypoint_file)
        return cls(
            robot=robot,
            store=store,
            lcm_client=lcm.LCM(lcm_url),
            cmd_channel=cmd_channel,
        )

    def serve_forever(self) -> None:
        self.store.load()
        if not self.robot.set_ready():
            raise RuntimeError("failed to set robot ready")

        self._subscription = self.lcm.subscribe(self.cmd_channel, self._on_command_message)
        self._command_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._command_thread.start()

        try:
            while not self._stop_event.is_set():
                self.lcm.handle_timeout(self.handle_timeout_ms)
        finally:
            self.close()

    def close(self) -> None:
        self._stop_event.set()
        if self._command_thread is not None and self._command_thread.is_alive():
            self._command_thread.join(timeout=1.0)
        motion_thread = self._motion_thread
        if motion_thread is not None and motion_thread.is_alive():
            motion_thread.join(timeout=1.0)

        try:
            self.robot.disconnect()
        except Exception:
            pass

    def enqueue_command(self, command: WaypointCommand) -> None:
        self._command_queue.put(command)

    def _on_command_message(self, channel: str, data: bytes) -> None:
        del channel
        try:
            command = WaypointCommand.decode(data)
        except Exception as exc:
            print(f"Invalid command payload: {exc}")
            return
        self.enqueue_command(command)

    def _command_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                command = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._handle_command(command)

    def _handle_command(self, command: WaypointCommand) -> None:
        op = command.op.strip().upper()
        if op == OP_TRACK_TCP:
            self._handle_track_tcp(command)
            return
        if op == OP_SAVE_CURRENT:
            self._handle_save_current(command)
            return
        if op == OP_GOTO:
            self._handle_goto(command)
            return
        if op == OP_HOME:
            self._handle_home()
            return
        print(f"Unsupported op: {command.op}")

    def _handle_track_tcp(self, command: WaypointCommand) -> None:
        if self.is_motion_busy():
            print("Robot is busy executing a joint motion")
            return

        tcp_delta = list(command.tcp_cmd)
        if len(tcp_delta) != TCP_COMMAND_DIM:
            print(f"Invalid tcp_cmd length: expected {TCP_COMMAND_DIM}, got {len(tcp_delta)}")
            return

        try:
            tcp_target = self._apply_tcp_delta(tcp_delta)
            res = self.robot.track_tcp_pose(tcp_target)
            if res is False:
                print("tcp delta command rejected")
                return
            if isinstance(res, dict) and res.get("recv") == "Task_Refuse":
                print(f"tcp delta command rejected: {res}")
        except Exception as exc:
            print(f"tcp delta command execution failed: {exc}")

    def _apply_tcp_delta(self, tcp_delta: list[float]) -> list[float]:
        with self._state_lock:
            tcp_target = list(self._tcp_target) if self._tcp_target is not None else None

        if tcp_target is None:
            tcp_target = self.robot.get_tcp_pose()
            if len(tcp_target) != TCP_COMMAND_DIM:
                raise RuntimeError("Current TCP pose is unavailable or invalid")

        tcp_target = [
            target_value + delta_value
            for target_value, delta_value in zip(tcp_target, tcp_delta, strict=True)
        ]
        clamp_tcp_target(tcp_target)

        with self._state_lock:
            self._tcp_target = list(tcp_target)

        return tcp_target

    def _handle_save_current(self, command: WaypointCommand) -> None:
        if self.is_motion_busy():
            print("Robot is busy executing a joint motion")
            return
        if not command.name:
            print("SAVE_CURRENT requires a waypoint name")
            return

        joint_pos = list(self.robot.joint_pos)
        if len(joint_pos) != 6:
            print("Current joint_pos is unavailable or invalid")
            return

        self.store.save(
            command.name,
            joint_pos=joint_pos,
        )

    def _handle_goto(self, command: WaypointCommand) -> None:
        if self.is_motion_busy():
            print("Robot is busy executing a joint motion")
            return
        if not command.name:
            print("GOTO requires a waypoint name")
            return

        record = self.store.get(command.name)
        if record is None:
            print(f"Waypoint not found: {command.name}")
            return

        self._start_motion(record.joint_pos, motion_name="GOTO")

    def _handle_home(self) -> None:
        if self.is_motion_busy():
            print("Robot is busy executing a joint motion")
            return
        self._start_motion(HOME_JOINTS, motion_name="HOME")

    def _start_motion(self, joint_pos: list[float], *, motion_name: str) -> None:
        self._motion_thread = threading.Thread(
            target=self._run_motion,
            args=(list(joint_pos), motion_name),
            daemon=True,
        )
        self._motion_thread.start()

    def _run_motion(self, joint_pos: list[float], motion_name: str) -> None:
        try:
            res = self.robot.move_joint(list(joint_pos), is_sync=True)
            if res.get("recv") != "Task_Recieve":
                print(f"{motion_name} rejected: {res}")
        except Exception as exc:
            print(f"{motion_name} execution failed: {exc}")
        finally:
            self._sync_tcp_target_from_robot(motion_name)
            with self._state_lock:
                self._motion_thread = None

    def is_motion_busy(self) -> bool:
        with self._state_lock:
            return self._motion_thread is not None and self._motion_thread.is_alive()

    def _sync_tcp_target_from_robot(self, motion_name: str) -> None:
        try:
            tcp_target = self.robot.get_tcp_pose()
            if len(tcp_target) != TCP_COMMAND_DIM:
                print(f"{motion_name} TCP target sync failed: invalid TCP pose")
                return
        except Exception as exc:
            print(f"{motion_name} TCP target sync failed: {exc}")
            return

        clamp_tcp_target(tcp_target)
        with self._state_lock:
            self._tcp_target = list(tcp_target)
