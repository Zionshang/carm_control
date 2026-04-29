#!/usr/bin/env python3
import argparse

from controller.lcm_waypoint_service import (
    COMMAND_CHANNEL,
    DEFAULT_LCM_URL,
    DEFAULT_WAYPOINT_FILE,
    LcmWaypointService,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Carm LCM tcp_cmd service")
    parser.add_argument("--addr", default="10.42.0.101", help="Robot controller IP")
    parser.add_argument("--lcm-url", default=DEFAULT_LCM_URL, help="LCM URL")
    parser.add_argument(
        "--waypoint-file",
        default=DEFAULT_WAYPOINT_FILE,
        help="Path to the waypoint JSON file",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    service = LcmWaypointService.from_robot_config(
        addr=args.addr,
        lcm_url=args.lcm_url,
        waypoint_file=args.waypoint_file,
    )
    print(f"Listening for tcp_cmd on channel: {COMMAND_CHANNEL}")
    service.serve_forever()


if __name__ == "__main__":
    main()
