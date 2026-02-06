# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Dual-arm MoveIt launch: N move_group nodes with namespaces from robot_names argument.
# Requires ur_sim_dual_control to be running (robot_state_publisher, controllers).
#
# Usage: ros2 launch ur_simulation_gz ur_dual_moveit.launch.py robot_names:=ur1,ur2

import os
import sys

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

# Import shared dual-arm utilities
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_arm_utils import (
    get_prefixed_controllers_path,
    get_prefixed_joint_limits,
    get_prefixed_moveit_controllers_path,
)


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    robot_names = LaunchConfiguration("robot_names", default="ur1,ur2")
    base_positions = LaunchConfiguration("base_positions", default="0 0 0, 1 0 0")

    sim_gz_share = get_package_share_directory("ur_simulation_gz")
    moveit_config_share = get_package_share_directory("ur_moveit_config")

    robot_names_val = context.perform_substitution(robot_names)
    robots = [r.strip() for r in robot_names_val.split(",") if r.strip()]

    base_positions_val = context.perform_substitution(base_positions)
    positions = [p.strip() for p in base_positions_val.split(",") if p.strip()]
    while len(positions) < len(robots):
        positions.append("0 0 0")

    ur_type_val = context.perform_substitution(ur_type)

    warehouse_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }

    moveit_configs = []
    move_group_nodes = []

    for i, name in enumerate(robots):
        tf_prefix = ""  # No prefix: joint names are elbow_joint, etc.; namespace disambiguates
        base_xyz = positions[i] if i < len(positions) else "0 0 0"

        controllers_path = get_prefixed_controllers_path(sim_gz_share, tf_prefix)
        moveit_controllers_path = get_prefixed_moveit_controllers_path(sim_gz_share, tf_prefix)

        moveit_config = (
            MoveItConfigsBuilder(robot_name=name, package_name="ur_simulation_gz")
            .robot_description(
                file_path="urdf/ur_gz.urdf.xacro",
                mappings={
                    "name": name,
                    "tf_prefix": tf_prefix,
                    "ur_type": ur_type_val,
                    "safety_limits": "true",
                    "safety_pos_margin": "0.15",
                    "safety_k_position": "20",
                    "ros_namespace": name,
                    "base_xyz": base_xyz,
                    "simulation_controllers": controllers_path,
                },
            )
            .robot_description_semantic(
                file_path="srdf/ur_dual_srdf.xacro",
                mappings={"name": name, "tf_prefix": tf_prefix},
            )
            .robot_description_kinematics(
                file_path=os.path.join(moveit_config_share, "config", "kinematics.yaml")
            )
            .joint_limits(file_path="config/joint_limits.yaml")
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .trajectory_execution(file_path=moveit_controllers_path)
            .to_moveit_configs()
        )

        prefixed_joint_limits = get_prefixed_joint_limits(sim_gz_share, tf_prefix)
        planning_key = "robot_description_planning"
        moveit_config.joint_limits[planning_key]["joint_limits"] = prefixed_joint_limits

        moveit_configs.append(moveit_config)

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=name,
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                warehouse_config,
                {"use_sim_time": use_sim_time},
                {"publish_robot_description_semantic": True},
            ],
            remappings=[
                ("/joint_states", f"/{name}/joint_states"),
            ],
        )
        move_group_nodes.append(move_group_node)

    rviz_config_file = os.path.join(sim_gz_share, "config", "moveit_dual.rviz")
    rviz_params = [
        moveit_configs[0].robot_description,
        moveit_configs[0].robot_description_semantic,
        moveit_configs[0].robot_description_kinematics,
        moveit_configs[0].planning_pipelines,
        moveit_configs[0].joint_limits,
        warehouse_config,
        {"use_sim_time": use_sim_time},
    ]
    for i in range(1, len(moveit_configs)):
        rviz_params.append(
            {
                f"robot_description_{robots[i]}": list(
                    moveit_configs[i].robot_description.values()
                )[0]
            }
        )
        rviz_params.append(
            {
                f"robot_description_{robots[i]}_semantic": list(
                    moveit_configs[i].robot_description_semantic.values()
                )[0]
            }
        )
        rviz_params.append(
            {
                f"robot_description_{robots[i]}_kinematics": list(
                    moveit_configs[i].robot_description_kinematics.values()
                )[0]
            }
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=rviz_params,
        remappings=[
            ("/joint_states", f"/{robots[0]}/joint_states"),
        ],
    )

    return move_group_nodes + [rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
        ),
        DeclareLaunchArgument(
            "robot_names",
            default_value="ur1,ur2",
            description="Comma-separated robot names (namespace + tf_prefix base).",
        ),
        DeclareLaunchArgument(
            "base_positions",
            default_value="0 0 0, 1 0 0",
            description="Comma-separated base xyz positions for each robot (e.g. '0 0 0, 1 0 0').",
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
