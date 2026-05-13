#!/usr/bin/env python3
import argparse

from controller.lcm_waypoint_service import (
    LcmWaypointService,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Carm LCM tcp_cmd service")
    parser.add_argument("--addr", default="192.168.100.100", help="Robot controller IP")
    parser.add_argument("--lcm-url", default="udpm://239.255.76.67:7667?ttl=1", help="LCM URL")
    parser.add_argument(
        "--waypoint-file",
        default="data/waypoints.json",
        help="Path to the waypoint JSON file",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    service = LcmWaypointService.from_robot_config(
        addr=args.addr,
        lcm_url=args.lcm_url,
        waypoint_file=args.waypoint_file,
        cmd_channel="tcp_cmd",
    )
    print(f"Listening for tcp_cmd channel")
    service.serve_forever()


if __name__ == "__main__":
    main()
