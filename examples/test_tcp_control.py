import math

from controller.tcp_carm import DEFAULT_TCP_OFFSET, TcpCarm


def main():
    robot = TcpCarm()
    robot.set_ready()
    robot.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], is_sync=True)

    robot.set_tcp_offset(DEFAULT_TCP_OFFSET)
    robot.move_tcp_pose([0.32000, 0.010000, 0.350000, 0.0, math.pi / 2, 0.0], is_sync=True)
    robot.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], is_sync=True)
    robot.disconnect()


if __name__ == "__main__":
    main()
