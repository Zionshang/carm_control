import math
from typing import Any, Sequence

from carm import Carm

from utils.rotation_utils import (
    euler_zyx_to_quat,
    quat_conjugate,
    quat_mul,
    quat_norm,
    quat_rotate_vector,
    quat_to_euler_zyx,
)

DEFAULT_TCP_OFFSET = [0.0, 0.0, 0.0, math.pi, -math.pi / 2, 0.0]


class TcpCarm(Carm):
    """
    带 TCP offset 支持的 Carm。

    6 维姿态统一使用 [x, y, z, roll, pitch, yaw]。
    欧拉角数组按 roll/pitch/yaw 存储，单位为弧度；
    旋转组合顺序为 ZYX: Rz(yaw) * Ry(pitch) * Rx(roll)。
    """

    _tcp_offset_pos: list[float]
    _tcp_offset_quat: list[float]

    def __init__(
        self,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        # self.on_task_finish(self._ignore_task_finish)
        self.set_tcp_offset(DEFAULT_TCP_OFFSET)

    @staticmethod
    def _ignore_task_finish(task_key: str) -> None:
        return None

    def set_tcp_offset(self, tcp_offset: Sequence[float]) -> None:
        """
        设置 TCP 到 flange 的固定偏移。

        tcp_offset:
          [x, y, z, roll, pitch, yaw]
        """
        if len(tcp_offset) != 6:
            raise ValueError("tcp_offset must be length 6")

        x, y, z, roll, pitch, yaw = tcp_offset
        self._tcp_offset_pos = [x, y, z]
        self._tcp_offset_quat = euler_zyx_to_quat([roll, pitch, yaw])

    def move_tcp_pose(
        self,
        tcp_pose: Sequence[float],
        is_sync: bool = True,
    ) -> dict:
        """
        移动 TCP 到目标 6 维姿态。

        tcp_pose:
          [x, y, z, roll, pitch, yaw]
        """
        flange_pose = self.tcp_pose_to_flange_pose(tcp_pose)
        return self.move_pose(flange_pose, tm=-1, is_sync=is_sync)

    def track_tcp_pose(
        self,
        tcp_pose: Sequence[float],
    ) -> dict | bool:
        """
        周期跟踪 TCP 的 6 维目标姿态。

        tcp_pose:
          [x, y, z, roll, pitch, yaw]
        """
        flange_pose = self.tcp_pose_to_flange_pose(tcp_pose)
        return self.track_pose(flange_pose)

    def tcp_pose_to_flange_pose(self, tcp_pose: Sequence[float]) -> list[float]:
        """
        将 TCP 的 6 维目标姿态转换为 flange 的 7 维目标姿态。

        tcp_pose:
          [x, y, z, roll, pitch, yaw]
        返回:
          [x, y, z, qx, qy, qz, qw]
        """
        if len(tcp_pose) != 6:
            raise ValueError("tcp_pose must be length 6")

        x, y, z, roll, pitch, yaw = tcp_pose
        q_tcp = euler_zyx_to_quat([roll, pitch, yaw])
        rotated_offset = quat_rotate_vector(q_tcp, self._tcp_offset_pos)
        flange_pos = [
            x + rotated_offset[0],
            y + rotated_offset[1],
            z + rotated_offset[2],
        ]
        flange_quat = quat_norm(quat_mul(q_tcp, self._tcp_offset_quat))
        return [*flange_pos, *flange_quat]

    def flange_pose_to_tcp_pose(self, flange_pose: Sequence[float]) -> list[float]:
        """
        将 flange 的 7 维姿态转换为 TCP 的 6 维姿态。

        flange_pose:
          [x, y, z, qx, qy, qz, qw]
        返回:
          [x, y, z, roll, pitch, yaw]
        """
        if len(flange_pose) != 7:
            raise ValueError("flange_pose must be length 7")

        x, y, z, qx, qy, qz, qw = flange_pose
        q_flange = quat_norm([qx, qy, qz, qw])
        q_tcp = quat_norm(quat_mul(q_flange, quat_conjugate(self._tcp_offset_quat)))
        rotated_offset = quat_rotate_vector(q_tcp, self._tcp_offset_pos)
        tcp_pos = [
            x - rotated_offset[0],
            y - rotated_offset[1],
            z - rotated_offset[2],
        ]
        return [*tcp_pos, *quat_to_euler_zyx(q_tcp)]

    def get_tcp_pose(self) -> list[float]:
        """
        获取当前 TCP 的 6 维实际姿态。

        返回:
          [x, y, z, roll, pitch, yaw]
        """
        if len(self.cart_pose) != 7:
            raise RuntimeError("cart_pose is not available")
        return self.flange_pose_to_tcp_pose(self.cart_pose)
