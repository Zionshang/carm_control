#!/usr/bin/env python3
import argparse

import lcm
from carm_waypoint import WaypointCommand

COMMAND_CHANNEL = "tcp_cmd"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send TCP or waypoint commands over LCM")
    parser.add_argument(
        "op",
        choices=["TRACK_TCP", "SAVE_CURRENT", "GOTO", "HOME"],
        help="Command operation",
    )
    parser.add_argument("--name", default="", help="Waypoint name for SAVE_CURRENT/GOTO")
    parser.add_argument(
        "--tcp-delta",
        nargs=6,
        type=float,
        metavar=("DX", "DY", "DZ", "DROLL", "DPITCH", "DYAW"),
        help="TCP pose delta [dx, dy, dz, droll, dpitch, dyaw] for TRACK_TCP",
    )
    parser.add_argument("--lcm-url", default="udpm://239.255.76.67:7667?ttl=1", help="LCM URL")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    lcm_client = lcm.LCM(args.lcm_url)

    if args.op == "TRACK_TCP" and args.tcp_delta is None:
        raise SystemExit("--tcp-delta is required for TRACK_TCP")

    command = WaypointCommand()
    command.op = args.op
    command.name = args.name
    command.tcp_cmd = list(args.tcp_delta or [])
    lcm_client.publish(COMMAND_CHANNEL, command.encode())


if __name__ == "__main__":
    main()
