#!/usr/bin/env python3
import argparse

import lcm
from carm_waypoint import WaypointCommand

from controller.lcm_waypoint_service import COMMAND_CHANNEL, DEFAULT_LCM_URL


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send TCP or waypoint commands over LCM")
    parser.add_argument(
        "op",
        choices=["TRACK_TCP", "SAVE_CURRENT", "GOTO", "HOME"],
        help="Command operation",
    )
    parser.add_argument("--name", default="", help="Waypoint name for SAVE_CURRENT/GOTO")
    parser.add_argument(
        "--tcp-cmd",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Target TCP pose [x, y, z, roll, pitch, yaw] for TRACK_TCP",
    )
    parser.add_argument("--lcm-url", default=DEFAULT_LCM_URL, help="LCM URL")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    lcm_client = lcm.LCM(args.lcm_url)

    if args.op == "TRACK_TCP" and args.tcp_cmd is None:
        raise SystemExit("--tcp-cmd is required for TRACK_TCP")

    command = WaypointCommand()
    command.op = args.op
    command.name = args.name
    command.tcp_cmd = list(args.tcp_cmd or [])
    lcm_client.publish(COMMAND_CHANNEL, command.encode())


if __name__ == "__main__":
    main()
