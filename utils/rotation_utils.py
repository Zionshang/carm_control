import math
from typing import Sequence


def quat_norm(q: Sequence[float]) -> list[float]:
    if len(q) != 4:
        raise ValueError("quaternion must be length 4")

    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return [v / n for v in q]


def quat_mul(q1: Sequence[float], q2: Sequence[float]) -> list[float]:
    if len(q1) != 4 or len(q2) != 4:
        raise ValueError("quaternion must be length 4")

    # quaternion format: [x, y, z, w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def euler_zyx_to_quat(euler_rpy: Sequence[float]) -> list[float]:
    """
    将 ZYX 欧拉角转换为 xyzw 四元数。

    euler_rpy:
      数组按 [roll, pitch, yaw] 存储，单位为弧度；
      旋转组合顺序为 Rz(yaw) * Ry(pitch) * Rx(roll)。
    """
    if len(euler_rpy) != 3:
        raise ValueError("euler_rpy must be length 3")

    roll, pitch, yaw = euler_rpy
    cz = math.cos(yaw * 0.5)
    sz = math.sin(yaw * 0.5)
    cy = math.cos(pitch * 0.5)
    sy = math.sin(pitch * 0.5)
    cx = math.cos(roll * 0.5)
    sx = math.sin(roll * 0.5)

    qz = [0.0, 0.0, sz, cz]
    qy = [0.0, sy, 0.0, cy]
    qx = [sx, 0.0, 0.0, cx]
    return quat_norm(quat_mul(quat_mul(qz, qy), qx))


def quat_to_euler_zyx(q: Sequence[float]) -> list[float]:
    """
    将 xyzw 四元数转换为 ZYX 欧拉角。

    返回值:
      [roll, pitch, yaw]，单位为弧度。
    """
    x, y, z, w = quat_norm(q)

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    return [roll, pitch, yaw]


def quat_conjugate(q: Sequence[float]) -> list[float]:
    x, y, z, w = quat_norm(q)
    return [-x, -y, -z, w]


def quat_rotate_vector(q: Sequence[float], v: Sequence[float]) -> list[float]:
    if len(v) != 3:
        raise ValueError("vector must be length 3")

    q = quat_norm(q)
    vx, vy, vz = v
    rotated = quat_mul(quat_mul(q, [vx, vy, vz, 0.0]), quat_conjugate(q))
    return rotated[:3]
