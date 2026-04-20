#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause
"""Add a static tabletop collision box to MoveIt planning scene (matches Gazebo Table model).

Runs namespaced (e.g. under /ur1) and calls apply_planning_scene on that namespace.
"""
from __future__ import annotations

import pathlib
import sys
from typing import Any, Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header


def _load_yaml(path: pathlib.Path) -> dict:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return data


class PublishTableCollision(Node):
    def __init__(self) -> None:
        super().__init__("publish_moveit_table_collision")
        self.declare_parameter("collision_config", "")
        self.declare_parameter("service_timeout_sec", 120.0)

        cfg_path = self.get_parameter("collision_config").get_parameter_value().string_value.strip()
        timeout = float(self.get_parameter("service_timeout_sec").value)

        if cfg_path:
            path = pathlib.Path(cfg_path).expanduser()
        else:
            path = pathlib.Path(get_package_share_directory("ur_simulation_gz")) / "config" / "moveit_table_collision_lab_table_one.yaml"

        if not path.is_file():
            self.get_logger().fatal(f"collision_config not found: {path}")
            raise SystemExit(1)

        data = _load_yaml(path)

        srv = "apply_planning_scene"
        self.get_logger().info(f"Waiting for service {srv} (timeout {timeout}s) ...")
        cli = self.create_client(ApplyPlanningScene, srv)
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().fatal(f"Service {srv} not available")
            raise SystemExit(2)

        req = ApplyPlanningScene.Request()
        req.scene = self._build_planning_scene(data)

        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            self.get_logger().fatal("apply_planning_scene: no response")
            raise SystemExit(3)
        if not res.success:
            self.get_logger().error("apply_planning_scene returned success=False")
            raise SystemExit(4)

        ids = []
        for o in self._iter_collision_specs(data):
            ids.append(o.get("collision_object_id", "?"))
        self.get_logger().info(
            f"Added planning scene collision object(s) {ids} "
            f"(frame {data.get('frame_id', 'world')})"
        )

    def _iter_collision_specs(self, data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Legacy single-object keys or explicit collision_objects list."""
        if "collision_objects" in data and isinstance(data["collision_objects"], list):
            return list(data["collision_objects"])
        legacy = {
            "collision_object_id": data.get("collision_object_id", "lab_table_top"),
            "frame_id": data.get("frame_id", "world"),
            "position": data["position"],
            "orientation": data.get("orientation", {}),
            "dimensions": data["dimensions"],
        }
        if isinstance(data.get("color"), dict):
            legacy["color"] = data["color"]
        out = [legacy]
        extra = data.get("extra_collision_objects")
        if isinstance(extra, list):
            out.extend(extra)
        return out

    def _box_collision_object(self, spec: dict) -> CollisionObject:
        frame_id = str(spec.get("frame_id", "world"))
        cid = str(spec["collision_object_id"])
        pos = spec["position"]
        ori = spec.get("orientation") or {}
        dims = spec["dimensions"]

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(dims[0]), float(dims[1]), float(dims[2])]

        obj = CollisionObject()
        obj.header = Header()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = frame_id
        obj.id = cid
        obj.operation = CollisionObject.ADD
        obj.pose = Pose()
        obj.pose.position.x = float(pos["x"])
        obj.pose.position.y = float(pos["y"])
        obj.pose.position.z = float(pos["z"])
        obj.pose.orientation.x = float(ori.get("x", 0.0))
        obj.pose.orientation.y = float(ori.get("y", 0.0))
        obj.pose.orientation.z = float(ori.get("z", 0.0))
        obj.pose.orientation.w = float(ori.get("w", 1.0))
        obj.primitives.append(prim)
        ppose = Pose()
        ppose.orientation.w = 1.0
        obj.primitive_poses.append(ppose)
        return obj

    def _build_planning_scene(self, data: dict) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        default_frame = str(data.get("frame_id", "world"))
        for spec in self._iter_collision_specs(data):
            merged = dict(spec)
            if "frame_id" not in merged:
                merged["frame_id"] = default_frame
            obj = self._box_collision_object(merged)
            scene.world.collision_objects.append(obj)
            color_spec = merged.get("color")
            if isinstance(color_spec, dict):
                oc = ObjectColor()
                oc.id = obj.id
                oc.color = ColorRGBA(
                    r=float(color_spec.get("r", 0.5)),
                    g=float(color_spec.get("g", 0.5)),
                    b=float(color_spec.get("b", 0.5)),
                    a=float(color_spec.get("a", 0.85)),
                )
                scene.object_colors.append(oc)
        return scene


def main() -> None:
    rclpy.init()
    node: PublishTableCollision | None = None
    try:
        node = PublishTableCollision()
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 1
        sys.exit(code)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
