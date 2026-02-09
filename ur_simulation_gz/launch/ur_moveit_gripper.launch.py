# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
# Modifications: MoveIt launch for UR + Robotiq gripper with custom SRDF

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic", default="true"
    )

    # Use ur_moveit_config for kinematics etc., but our SRDF with gripper disable_collisions
    srdf_path = Path(get_package_share_directory("ur_simulation_gz")) / "srdf" / "ur_gripper.srdf.xacro"

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(str(srdf_path), {"name": ur_type, "tf_prefix": ""})
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "ur_type",
            description="UR robot type",
            choices=["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30"],
            default_value="ur5e",
        )
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    ld.add_action(wait_robot_description)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node],
            )
        )
    )

    return ld
