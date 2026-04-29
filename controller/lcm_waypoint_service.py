from __future__ import annotations

import queue
import threading
from typing import Any, Protocol

import lcm
from carm_waypoint import WaypointCommand

from controller.tcp_carm import TcpCarm
from controller.waypoint_store import WaypointStore

DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=1"
COMMAND_CHANNEL = "tcp_cmd"
DEFAULT_WAYPOINT_FILE = "data/waypoints.json"

OP_TRACK_TCP = "TRACK_TCP"
OP_SAVE_CURRENT = "SAVE_CURRENT"
OP_GOTO = "GOTO"
OP_STOP = "STOP"


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
        cmd_channel: str = COMMAND_CHANNEL,
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
        self._goto_thread: threading.Thread | None = None

    @classmethod
    def from_robot_config(
        cls,
        *,
        addr: str = "10.42.0.101",
        lcm_url: str = DEFAULT_LCM_URL,
        waypoint_file: str = DEFAULT_WAYPOINT_FILE,
    ) -> "LcmWaypointService":
        robot = TcpCarm(addr=addr)
        store = WaypointStore(waypoint_file)
        return cls(robot=robot, store=store, lcm_client=lcm.LCM(lcm_url))

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
        goto_thread = self._goto_thread
        if goto_thread is not None and goto_thread.is_alive():
            goto_thread.join(timeout=1.0)

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
        if op == OP_STOP:
            self._handle_stop()
            return
        print(f"Unsupported op: {command.op}")

    def _handle_track_tcp(self, command: WaypointCommand) -> None:
        if self.is_goto_busy():
            print("Robot is busy executing GOTO")
            return

        tcp_cmd = list(command.tcp_cmd)
        if len(tcp_cmd) != 6:
            print(f"Invalid tcp_cmd length: expected 6, got {len(tcp_cmd)}")
            return

        try:
            res = self.robot.track_tcp_pose(tcp_cmd)
            if res is False:
                print("tcp_cmd rejected")
                return
            if isinstance(res, dict) and res.get("recv") == "Task_Refuse":
                print(f"tcp_cmd rejected: {res}")
        except Exception as exc:
            print(f"tcp_cmd execution failed: {exc}")

    def _handle_save_current(self, command: WaypointCommand) -> None:
        if self.is_goto_busy():
            print("Robot is busy executing GOTO")
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
        if self.is_goto_busy():
            print("Robot is busy executing GOTO")
            return
        if not command.name:
            print("GOTO requires a waypoint name")
            return

        record = self.store.get(command.name)
        if record is None:
            print(f"Waypoint not found: {command.name}")
            return

        self._goto_thread = threading.Thread(
            target=self._run_goto,
            args=(record.joint_pos,),
            daemon=True,
        )
        self._goto_thread.start()

    def _run_goto(self, joint_pos: list[float]) -> None:
        try:
            res = self.robot.move_joint(list(joint_pos), is_sync=True)
            if res.get("recv") != "Task_Recieve":
                print(f"GOTO rejected: {res}")
        except Exception as exc:
            print(f"GOTO execution failed: {exc}")
        finally:
            with self._state_lock:
                self._goto_thread = None

    def _handle_stop(self) -> None:
        res = self.robot.stop_task(at_once=True)
        if res.get("recv") != "Task_Recieve":
            print(f"Stop rejected: {res}")

    def is_goto_busy(self) -> bool:
        with self._state_lock:
            return self._goto_thread is not None and self._goto_thread.is_alive()
