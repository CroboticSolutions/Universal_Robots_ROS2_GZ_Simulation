#!/usr/bin/env python
import time
import unittest

import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory


SERVICE_TIMEOUT = 90.0
ACTION_TIMEOUT = 90.0


def _wait_for_service(node, service_name, service_type, timeout):
    client = node.create_client(service_type, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"Service not reachable: {service_name}")
    return client


def _wait_for_action(node, action_name, action_type, timeout):
    client = ActionClient(node, action_type, action_name)
    if not client.wait_for_server(timeout_sec=timeout):
        raise RuntimeError(f"Action server not reachable: {action_name}")
    return client


@pytest.mark.launch_test
def generate_test_description():
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "multi_ur_sim_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_count": "2",
            "robot_positions": "0,0,0,0;1.5,0,0,1.57",
            "gazebo_gui": "false",
            "launch_rviz_first_robot": "false",
        }.items(),
    )
    return LaunchDescription([ReadyToTest(), launch_file])


class MultiUrLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("multi_ur_launch_test")
        # Give launch graph a short warmup before waiting on endpoints.
        time.sleep(2)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_controller_managers_reachable(self):
        _wait_for_service(
            self.node,
            "/ur1/controller_manager/list_controllers",
            ListControllers,
            SERVICE_TIMEOUT,
        )
        _wait_for_service(
            self.node,
            "/ur2/controller_manager/list_controllers",
            ListControllers,
            SERVICE_TIMEOUT,
        )

    def test_trajectory_actions_reachable(self):
        _wait_for_action(
            self.node,
            "/ur1/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
            ACTION_TIMEOUT,
        )
        _wait_for_action(
            self.node,
            "/ur2/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
            ACTION_TIMEOUT,
        )
