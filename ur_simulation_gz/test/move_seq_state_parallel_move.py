#!/usr/bin/env python3
"""
Set robots to CART_TRAJ_CTL in parallel, then send Cartesian goals in parallel.

This utility uses ROS 2 CLI commands so it can run without extra package deps.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import subprocess
import sys
from dataclasses import dataclass


@dataclass
class PoseTarget:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def run_cmd(cmd: list[str], timeout_s: float) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        return proc.returncode, proc.stdout, proc.stderr
    except subprocess.TimeoutExpired as exc:
        return 124, exc.stdout or "", (exc.stderr or "") + "\nTIMEOUT"


def build_goal_json(pose: PoseTarget) -> str:
    payload = {
        "goal": {
            "header": {"frame_id": "world"},
            "pose": {
                "position": {"x": pose.x, "y": pose.y, "z": pose.z},
                "orientation": {
                    "x": pose.qx,
                    "y": pose.qy,
                    "z": pose.qz,
                    "w": pose.qw,
                },
            },
        }
    }
    return json.dumps(payload, separators=(",", ":"))


def set_cart_state(ns: str, timeout_s: float) -> tuple[bool, str]:
    cmd = [
        "ros2",
        "service",
        "call",
        f"/{ns}/arm/change_state",
        "arm_api2_msgs/srv/ChangeState",
        "{state: CART_TRAJ_CTL}",
    ]
    rc, out, err = run_cmd(cmd, timeout_s=timeout_s)
    ok = rc == 0 and "success=True" in out
    msg = out.strip() if out.strip() else err.strip()
    return ok, msg


def send_move_goal(ns: str, goal_json: str, timeout_s: float) -> tuple[bool, str]:
    cmd = [
        "ros2",
        "action",
        "send_goal",
        f"/{ns}/arm/move_to_pose",
        "arm_api2_msgs/action/MoveCartesian",
        goal_json,
    ]
    rc, out, err = run_cmd(cmd, timeout_s=timeout_s)
    ok = rc == 0
    msg = out.strip() if out.strip() else err.strip()
    return ok, msg


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Parallel CART_TRAJ_CTL state switch, parallel MoveCartesian goals."
    )
    parser.add_argument("--count", type=int, default=10, help="Robot count (default: 10).")
    parser.add_argument("--x", type=float, default=0.05, help="Target X (world frame).")
    parser.add_argument("--y", type=float, default=0.28, help="Target Y (world frame).")
    parser.add_argument("--z", type=float, default=1.03, help="Target Z in world frame.")
    parser.add_argument("--qx", type=float, default=-0.70710656)
    parser.add_argument("--qy", type=float, default=0.00056309)
    parser.add_argument("--qz", type=float, default=0.00056308)
    parser.add_argument("--qw", type=float, default=0.70710656)
    parser.add_argument(
        "--service-timeout", type=float, default=8.0, help="Timeout for state service calls."
    )
    parser.add_argument(
        "--action-timeout", type=float, default=45.0, help="Timeout for action goal command."
    )
    args = parser.parse_args()

    if args.count < 1:
        print("count must be >= 1", file=sys.stderr)
        return 2

    namespaces = [f"ur{i}" for i in range(1, args.count + 1)]

    print(f"==> Step 1: Parallel state change to CART_TRAJ_CTL ({args.count} robots)")
    state_ok: dict[str, bool] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.count) as pool:
        future_to_ns = {
            pool.submit(set_cart_state, ns, args.service_timeout): ns for ns in namespaces
        }
        for future in concurrent.futures.as_completed(future_to_ns):
            ns = future_to_ns[future]
            try:
                ok, msg = future.result()
            except Exception as exc:  # pylint: disable=broad-except
                ok, msg = False, f"Exception: {exc}"
            state_ok[ns] = ok
            print(f"[{ns}] change_state: {'OK' if ok else 'FAIL'}")
            if not ok and msg:
                print(f"  {msg}")

    pose = PoseTarget(
        x=args.x,
        y=args.y,
        z=args.z,
        qx=args.qx,
        qy=args.qy,
        qz=args.qz,
        qw=args.qw,
    )
    goal_json = build_goal_json(pose)

    print("==> Step 2: Parallel move_to_pose goals")
    failures = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.count) as pool:
        future_map = {}
        for ns in namespaces:
            ok = state_ok.get(ns, False)
            if not ok:
                print(f"[{ns}] skip move_to_pose (state transition failed)")
                failures += 1
                continue
            future = pool.submit(send_move_goal, ns, goal_json, args.action_timeout)
            future_map[future] = ns

        for future in concurrent.futures.as_completed(future_map):
            ns = future_map[future]
            try:
                ok, msg = future.result()
            except Exception as exc:  # pylint: disable=broad-except
                ok, msg = False, f"Exception: {exc}"
            print(f"[{ns}] move_to_pose: {'OK' if ok else 'FAIL'}")
            if not ok:
                failures += 1
                if msg:
                    print(f"  {msg}")

    print("==> Done.")
    return 1 if failures > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
