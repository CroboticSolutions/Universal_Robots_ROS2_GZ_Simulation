# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Dual-arm MoveIt launch: 2 move_group nodes (ur1, ur2) with namespaces.
# Requires ur_sim_dual_control to be running (robot_state_publisher, controllers).

import os

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path) as f:
            return __import__("yaml").safe_load(f)
    except OSError:
        return None


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    sim_gz_share = get_package_share_directory("ur_simulation_gz")
    moveit_config_share = get_package_share_directory("ur_moveit_config")

    controllers_ur1 = os.path.join(sim_gz_share, "config", "ur_controllers_sim_ur1.yaml")
    controllers_ur2 = os.path.join(sim_gz_share, "config", "ur_controllers_sim_ur2.yaml")

    ur_type_val = context.perform_substitution(ur_type)

    # MoveIt config for ur1 (paths relative to ur_simulation_gz share)
    moveit_config_ur1 = (
        MoveItConfigsBuilder(robot_name="ur1", package_name="ur_simulation_gz")
        .robot_description(
            file_path="urdf/ur_gz.urdf.xacro",
            mappings={
                "name": "ur1",
                "tf_prefix": "ur1_",
                "ur_type": ur_type_val,
                "safety_limits": "true",
                "safety_pos_margin": "0.15",
                "safety_k_position": "20",
                "ros_namespace": "ur1",
                "base_xyz": "0 0 0",
                "simulation_controllers": controllers_ur1,
            },
        )
        .robot_description_semantic(file_path="srdf/ur_dual_srdf.xacro", mappings={"name": "ur1", "tf_prefix": "ur1_"})
        .robot_description_kinematics(file_path=os.path.join(moveit_config_share, "config", "kinematics.yaml"))
        .joint_limits(file_path="config/joint_limits_ur1.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .trajectory_execution(file_path="config/moveit_controllers_ur1.yaml")
        .to_moveit_configs()
    )

    # MoveIt config for ur2
    moveit_config_ur2 = (
        MoveItConfigsBuilder(robot_name="ur2", package_name="ur_simulation_gz")
        .robot_description(
            file_path="urdf/ur_gz.urdf.xacro",
            mappings={
                "name": "ur2",
                "tf_prefix": "ur2_",
                "ur_type": ur_type_val,
                "safety_limits": "true",
                "safety_pos_margin": "0.15",
                "safety_k_position": "20",
                "ros_namespace": "ur2",
                "base_xyz": "1 0 0",
                "simulation_controllers": controllers_ur2,
            },
        )
        .robot_description_semantic(file_path="srdf/ur_dual_srdf.xacro", mappings={"name": "ur2", "tf_prefix": "ur2_"})
        .robot_description_kinematics(file_path=os.path.join(moveit_config_share, "config", "kinematics.yaml"))
        .joint_limits(file_path="config/joint_limits_ur2.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .trajectory_execution(file_path="config/moveit_controllers_ur2.yaml")
        .to_moveit_configs()
    )

    warehouse_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }

    move_group_ur1 = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="ur1",
        output="screen",
        parameters=[
            moveit_config_ur1.to_dict(),
            warehouse_config,
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
        remappings=[
            ("/joint_states", "/ur1/joint_states"),
        ],
    )

    move_group_ur2 = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="ur2",
        output="screen",
        parameters=[
            moveit_config_ur2.to_dict(),
            warehouse_config,
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
        remappings=[
            ("/joint_states", "/ur2/joint_states"),
        ],
    )

    rviz_config_file = os.path.join(sim_gz_share, "config", "moveit_dual.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config_ur1.robot_description,
            moveit_config_ur1.robot_description_semantic,
            moveit_config_ur1.robot_description_kinematics,
            moveit_config_ur1.planning_pipelines,
            moveit_config_ur1.joint_limits,
            {"robot_description_ur2": list(moveit_config_ur2.robot_description.values())[0]},
            {"robot_description_ur2_semantic": list(moveit_config_ur2.robot_description_semantic.values())[0]},
            {"robot_description_ur2_kinematics": list(moveit_config_ur2.robot_description_kinematics.values())[0]},
            warehouse_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/joint_states", "/ur1/joint_states"),
        ],
    )

    return [move_group_ur1, move_group_ur2, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
