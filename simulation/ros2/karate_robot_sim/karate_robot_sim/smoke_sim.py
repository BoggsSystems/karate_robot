#!/usr/bin/env python3
"""
Lightweight smoke test for the simulation launch path.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import time


def main() -> None:
    parser = argparse.ArgumentParser(description="Smoke test a Sensei launch file.")
    parser.add_argument(
        "--launch",
        default="sensei_phase1.launch.py",
        help="Launch file to run from karate_robot_sim.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to let the launch run before terminating.",
    )
    parser.add_argument(
        "--ros2-control",
        action="store_true",
        help="Enable ros2_control if the launch supports it.",
    )
    args = parser.parse_args()

    log_dir = os.environ.get("ROS_LOG_DIR")
    if not log_dir:
        log_dir = str(Path.cwd() / ".ros_logs")
        os.environ["ROS_LOG_DIR"] = log_dir
    Path(log_dir).mkdir(parents=True, exist_ok=True)

    launch_args = []
    if args.ros2_control:
        launch_args.append("enable_ros2_control:=true")

    cmd = [
        "ros2",
        "launch",
        "karate_robot_sim",
        args.launch,
        *launch_args,
    ]

    proc = subprocess.Popen(cmd)
    try:
        time.sleep(max(args.duration, 0.5))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    main()
