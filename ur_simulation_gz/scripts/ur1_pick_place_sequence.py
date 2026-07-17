#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Sekvenca za arm_api2 (moveit2_iface) u ROS namespaceu npr. ur1.

Koristi:
  - arm_api2_msgs/srv/ChangeState (JOINT_TRAJ_CTL / CART_TRAJ_CTL)
  - geometry_msgs/PoseStamped na arm/cmd/pose (isti topic u oba moda; različit planer u čvoru)
  - std_srvs/Trigger na arm/open_gripper i arm/close_gripper

Pokretanje (nakon multi_ur + arm_api2 s robot_namespaces:=ur1):
  ros2 run ur_simulation_gz ur1_pick_place_sequence --ros-args \\
    -p waypoint_file:=$(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/config/ur1_pick_place_waypoints.yaml

Ili:
  ./ur1_pick_place_sequence.py --ros-args -p waypoint_file:=/apsolutna/staza/waypoints.yaml

Napomena: čekanje između koraka je fiksno (seconds_per_move iz YAML-a ili parametra).
"""

from __future__ import annotations

import pathlib
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from arm_api2_msgs.srv import ChangeState
except ImportError:
    print("Paket arm_api2_msgs nije u PYTHONPATH-u. Sourceaj workspace (install/setup.bash).", file=sys.stderr)
    sys.exit(1)


def _load_waypoints(path: pathlib.Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _pose_from_dict(entry: dict) -> Pose:
    p = Pose()
    pos = entry["position"]
    ori = entry["orientation"]
    p.position.x = float(pos["x"])
    p.position.y = float(pos["y"])
    p.position.z = float(pos["z"])
    p.orientation.x = float(ori["x"])
    p.orientation.y = float(ori["y"])
    p.orientation.z = float(ori["z"])
    p.orientation.w = float(ori["w"])
    return p


class Ur1PickPlaceSequence(Node):
    def __init__(self) -> None:
        super().__init__("ur1_pick_place_sequence")

        self.declare_parameter("robot_namespace", "ur1")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("seconds_per_move", 0.0)

        ns = self.get_parameter("robot_namespace").get_parameter_value().string_value.strip().strip("/")
        waypoint_param = self.get_parameter("waypoint_file").get_parameter_value().string_value
        if waypoint_param:
            waypoint_path = pathlib.Path(waypoint_param).expanduser()
        else:
            waypoint_path = pathlib.Path(get_package_share_directory("ur_simulation_gz")) / "config" / "ur1_pick_place_waypoints.yaml"

        if not waypoint_path.is_file():
            self.get_logger().fatal(f"waypoint_file ne postoji: {waypoint_path}")
            raise SystemExit(1)

        data = _load_waypoints(waypoint_path)
        self._frame = data.get("planning_frame", "world")
        yaml_wait = float(data.get("seconds_per_move", 12.0))
        p_wait = float(self.get_parameter("seconds_per_move").value)
        self._wait = p_wait if p_wait > 0.0 else yaml_wait

        poses = data["poses"]
        for key in ("predrop0", "drop0", "postdrop0", "predrop1", "drop1", "postdrop1"):
            if key not in poses:
                self.get_logger().fatal(f"U YAML-u nedostaje poses.{key}")
                raise SystemExit(1)

        prefix = f"/{ns}/" if ns else "/"
        self._pub = self.create_publisher(PoseStamped, f"{prefix}arm/cmd/pose", 1)
        self._cli_change = self.create_client(ChangeState, f"{prefix}arm/change_state")
        self._cli_open = self.create_client(Trigger, f"{prefix}arm/open_gripper")
        self._cli_close = self.create_client(Trigger, f"{prefix}arm/close_gripper")

        self.get_logger().info(f"Učitano {waypoint_path}, frame={self._frame}, wait={self._wait}s, ns={ns or '(root)'}")

        self._poses = poses

        self._run_sequence()

    def _wait_srv(self, client, timeout_sec: float = 10.0) -> None:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Servis nije dostupan: {client.srv_name}")

    def _call_change(self, state_name: str) -> None:
        self._wait_srv(self._cli_change)
        req = ChangeState.Request()
        req.state = state_name
        fut = self._cli_change.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            raise RuntimeError(f"ChangeState({state_name}) nema odgovora")
        if not res.success:
            raise RuntimeError(f"ChangeState({state_name}) success=False")

    def _publish_pose_named(self, name: str) -> None:
        pose = _pose_from_dict(self._poses[name])
        msg = PoseStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        self._pub.publish(msg)
        self.get_logger().info(f"Poslana pozicija arm/cmd/pose: {name}")

    def _sleep_move(self) -> None:
        import time

        time.sleep(self._wait)

    def _trigger(self, client) -> None:
        self._wait_srv(client)
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            raise RuntimeError(f"Trigger {client.srv_name} nema odgovora")
        if not res.success:
            msg = getattr(res, "message", "")
            raise RuntimeError(f"Trigger {client.srv_name} success=False: {msg}")

    def _run_sequence(self) -> None:
        import time

        time.sleep(1.0)
        # 1
        self.get_logger().info("1. JOINT_TRAJ_CTL")
        self._call_change("JOINT_TRAJ_CTL")
        # 2
        self.get_logger().info("2. predrop0")
        self._publish_pose_named("predrop0")
        self._sleep_move()
        # 3
        self.get_logger().info("3. CART_TRAJ_CTL")
        self._call_change("CART_TRAJ_CTL")
        # 4
        self.get_logger().info("4. drop0")
        self._publish_pose_named("drop0")
        self._sleep_move()
        # 5
        self.get_logger().info("5. close_gripper")
        self._trigger(self._cli_close)
        # 6
        self.get_logger().info("6. postdrop0")
        self._publish_pose_named("postdrop0")
        self._sleep_move()
        # 7
        self.get_logger().info("7. JOINT_TRAJ_CTL")
        self._call_change("JOINT_TRAJ_CTL")
        # 8
        self.get_logger().info("8. predrop1")
        self._publish_pose_named("predrop1")
        self._sleep_move()
        # 9
        self.get_logger().info("9. CART_TRAJ_CTL")
        self._call_change("CART_TRAJ_CTL")
        # 10
        self.get_logger().info("10. drop1")
        self._publish_pose_named("drop1")
        self._sleep_move()
        # 11
        self.get_logger().info("11. open_gripper")
        self._trigger(self._cli_open)
        # 12
        self.get_logger().info("12. postdrop1")
        self._publish_pose_named("postdrop1")
        self._sleep_move()

        self.get_logger().info("Sekvenca završena.")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = Ur1PickPlaceSequence()
    except SystemExit:
        raise
    except Exception as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
