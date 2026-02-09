#!/usr/bin/env python3
# Copyright 2024 Universal Robots
#
# Publishes gripper commands. With position interface (Bullet): forwards goal directly.
# With effort interface (Dartsim): converts position to effort via P controller.
# Use: ros2 topic pub /gripper_position_goal std_msgs/msg/Float64 "{data: 0.8}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


class GripperCommandNode(Node):
    """Publishes gripper commands. Position mode: forward goal. Effort mode: P controller."""

    GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
    KP = 20.0  # Nm/rad (effort mode only)
    EFFORT_MAX = 15.0
    EFFORT_MIN = -15.0

    def __init__(self):
        super().__init__("gripper_command_node")
        self.declare_parameter("use_position_interface", True)
        self.declare_parameter("kp", self.KP)
        self.declare_parameter("effort_max", self.EFFORT_MAX)
        self.declare_parameter("effort_min", self.EFFORT_MIN)
        self.declare_parameter("publish_rate", 50.0)

        self.use_position = self.get_parameter("use_position_interface").value
        self.kp = self.get_parameter("kp").value
        self.effort_max = self.get_parameter("effort_max").value
        self.effort_min = self.get_parameter("effort_min").value
        pub_rate = self.get_parameter("publish_rate").value

        self.position_goal = 0.0
        self.current_position = 0.0
        self.has_joint_state = False
        self._last_log_time = 0.0

        self.sub_goal = self.create_subscription(
            Float64,
            "gripper_position_goal",
            self._goal_cb,
            10,
        )
        self.sub_joint_state = self.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_cb,
            10,
        )
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            "gripper_controller/commands",
            10,
        )
        self.timer = self.create_timer(1.0 / pub_rate, self._publish_cmd)

    def _goal_cb(self, msg: Float64):
        prev = self.position_goal
        self.position_goal = msg.data
        if prev != self.position_goal:
            self.get_logger().info(f"gripper_position_goal received: {self.position_goal:.2f}")

    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index(self.GRIPPER_JOINT)
            self.current_position = msg.position[idx]
            self.has_joint_state = True
        except (ValueError, IndexError):
            pass

    def _log_throttle(self, msg: str, interval: float = 2.0) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time >= interval:
            self.get_logger().info(msg)
            self._last_log_time = now

    def _publish_cmd(self):
        if self.use_position:
            msg = Float64MultiArray(data=[self.position_goal])
            self.pub_cmd.publish(msg)
            self._log_throttle(f"gripper: goal={self.position_goal:.2f} (position)")
            return
        if not self.has_joint_state:
            return
        error = self.position_goal - self.current_position
        effort = self.kp * error
        effort = max(self.effort_min, min(self.effort_max, effort))
        msg = Float64MultiArray(data=[effort])
        self.pub_cmd.publish(msg)
        self._log_throttle(
            f"gripper: goal={self.position_goal:.2f} pos={self.current_position:.3f} "
            f"effort={effort:.2f} Nm",
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
