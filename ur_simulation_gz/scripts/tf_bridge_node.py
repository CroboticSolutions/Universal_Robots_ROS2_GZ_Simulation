#!/usr/bin/env python3
# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# TF bridge: subscribes to namespaced /ur1/tf, /ur2/tf and republishes to /tf
# with frame prefixes for RViz visualization. Enables joint names without prefix
# (elbow_joint) in joint_states while keeping unique frame IDs in global /tf.

import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

TF_STATIC_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class TFBridgeNode(Node):
    """Bridges namespaced TF topics to global /tf with frame prefixes."""

    def __init__(self, robot_names, base_positions):
        super().__init__("tf_bridge")
        self.declare_parameter("robot_names", robot_names)
        self.declare_parameter("base_positions", base_positions)

        robot_names_val = self.get_parameter("robot_names").get_parameter_value().string_value
        self.robots = [r.strip() for r in robot_names_val.split(",") if r.strip()]

        base_positions_val = self.get_parameter("base_positions").get_parameter_value().string_value
        positions = [p.strip() for p in base_positions_val.split(",") if p.strip()]
        while len(positions) < len(self.robots):
            positions.append("0 0 0")
        self.base_positions = []
        for p in positions[: len(self.robots)]:
            parts = p.split()
            if len(parts) >= 3:
                self.base_positions.append((float(parts[0]), float(parts[1]), float(parts[2])))
            else:
                self.base_positions.append((0.0, 0.0, 0.0))

        self.broadcaster = self.create_publisher(TFMessage, "/tf", 10)
        self.static_broadcaster = self.create_publisher(
            TFMessage, "/tf_static", TF_STATIC_QOS
        )

        for i, name in enumerate(self.robots):
            prefix = name + "_"
            self.create_subscription(
                TFMessage,
                f"/{name}/tf",
                lambda msg, p=prefix: self.tf_callback(msg, p),
                10,
            )
            self.create_subscription(
                TFMessage,
                f"/{name}/tf_static",
                lambda msg, p=prefix: self.tf_static_callback(msg, p),
                TF_STATIC_QOS,
            )

        self.static_published = False
        self.static_timer = self.create_timer(1.0, self.publish_static_transforms)

    def prefix_transform(self, transform, prefix):
        """Add prefix to frame_id and child_frame_id."""
        out = TransformStamped()
        out.header = transform.header
        out.child_frame_id = transform.child_frame_id
        out.transform = transform.transform

        out.header.frame_id = prefix + transform.header.frame_id if transform.header.frame_id != "" else ""
        out.child_frame_id = prefix + transform.child_frame_id if transform.child_frame_id != "" else ""
        return out

    def tf_callback(self, msg, prefix):
        """Republish TF with prefix."""
        out_msg = TFMessage()
        for t in msg.transforms:
            out_msg.transforms.append(self.prefix_transform(t, prefix))
        if out_msg.transforms:
            self.broadcaster.publish(out_msg)

    def tf_static_callback(self, msg, prefix):
        """Republish static TF with prefix."""
        out_msg = TFMessage()
        for t in msg.transforms:
            out_msg.transforms.append(self.prefix_transform(t, prefix))
        if out_msg.transforms:
            self.static_broadcaster.publish(out_msg)

    def publish_static_transforms(self):
        """Publish world -> ur1_world, ur2_world for RViz fixed frame.
        Both robot worlds are at global origin; base_xyz is in each robot's URDF.
        """
        if self.static_published:
            return
        self.static_published = True

        msg = TFMessage()
        for name in self.robots:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = name + "_world"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            msg.transforms.append(t)
        if msg.transforms:
            self.static_broadcaster.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot_names = "ur1,ur2"
    base_positions = "0 0 0, 1 0 0"
    for i, arg in enumerate(sys.argv):
        if arg.startswith("robot_names:="):
            robot_names = arg.split(":=", 1)[1]
        elif arg.startswith("base_positions:="):
            base_positions = arg.split(":=", 1)[1].strip('"')

    node = TFBridgeNode(robot_names, base_positions)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
