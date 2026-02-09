# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source and binary forms must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without prior written permission.
#
# Dual-arm variant: 1 world, N robots with namespaces from robot_names argument.
#
# Usage: ros2 launch ur_simulation_gz ur_sim_dual_control.launch.py robot_names:=ur1,ur2

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

# Import shared dual-arm utilities
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_arm_utils import get_prefixed_controllers_path


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_file = LaunchConfiguration("description_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    robot_names = LaunchConfiguration("robot_names", default="ur1,ur2")
    base_positions = LaunchConfiguration("base_positions", default="0 0 0, 1 0 0")

    pkg_share = get_package_share_directory("ur_simulation_gz")

    robot_names_val = context.perform_substitution(robot_names)
    robots = [r.strip() for r in robot_names_val.split(",") if r.strip()]

    base_positions_val = context.perform_substitution(base_positions)
    positions = [p.strip() for p in base_positions_val.split(",") if p.strip()]
    while len(positions) < len(robots):
        positions.append("0 0 0")

    robot_descriptions = []
    robot_state_publishers = []
    gz_spawn_nodes = []
    controller_paths = []

    for i, name in enumerate(robots):
        tf_prefix = ""  # No prefix: joint names are elbow_joint, etc.; namespace /ur1 disambiguates
        base_xyz = positions[i] if i < len(positions) else "0 0 0"

        controllers_path = get_prefixed_controllers_path(pkg_share, tf_prefix)
        controller_paths.append(controllers_path)

        robot_description = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                description_file,
                " ",
                "safety_limits:=",
                safety_limits,
                " ",
                "safety_pos_margin:=",
                safety_pos_margin,
                " ",
                "safety_k_position:=",
                safety_k_position,
                " ",
                f"name:={name}",
                " ",
                "ur_type:=",
                ur_type,
                " ",
                f"tf_prefix:={tf_prefix}",
                " ",
                f"ros_namespace:={name}",
                " ",
                f'base_xyz:="{base_xyz}"',
                " ",
                "simulation_controllers:=",
                TextSubstitution(text=controllers_path),
            ]
        )
        robot_descriptions.append(robot_description)

        robot_state_publishers.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=name,
                output="both",
                parameters=[{"use_sim_time": True}, {"robot_description": robot_description}],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            )
        )

        gz_spawn_nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-string",
                    robot_description,
                    "-name",
                    name,
                    "-allow_renaming",
                    "true",
                ],
            )
        )

    # Gazebo launch (single world)
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            )
        }.items(),
    )

    spawner_common = ["--controller-manager-timeout", "30", "--switch-timeout", "30"]

    joint_state_broadcasters = []
    scaled_joint_trajectory_spawners = []
    forward_position_spawners = []

    for i, name in enumerate(robots):
        joint_state_broadcasters.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c",
                    f"/{name}/controller_manager",
                    "-p",
                    controller_paths[i],
                ]
                + spawner_common,
            )
        )
        scaled_joint_trajectory_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "scaled_joint_trajectory_controller",
                    "-c",
                    f"/{name}/controller_manager",
                    "-p",
                    controller_paths[i],
                ]
                + spawner_common,
            )
        )
        forward_position_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "forward_position_controller",
                    "-c",
                    f"/{name}/controller_manager",
                    "--inactive",
                ]
                + spawner_common,
            )
        )

    # Clock bridge
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # TF bridge: merges /ur1/tf, /ur2/tf to /tf with frame prefixes for RViz
    tf_bridge_node = Node(
        package="ur_simulation_gz",
        executable="tf_bridge_node.py",
        name="tf_bridge",
        output="log",
        parameters=[
            {"use_sim_time": True},
            {"robot_names": robot_names_val},
            {"base_positions": base_positions_val},
        ],
        arguments=[
            f"robot_names:={robot_names_val}",
            f'base_positions:="{base_positions_val}"',
        ],
    )

    # Chain: spawn robot 0 -> spawn robot 1 -> ... -> delay 8s -> joint_state 0 -> ... -> scaled 0 -> ... -> forward 0 -> ...
    actions = [gz_launch_description, gz_sim_bridge, tf_bridge_node] + robot_state_publishers + [gz_spawn_nodes[0]]

    prev_spawn = gz_spawn_nodes[0]
    for i in range(1, len(gz_spawn_nodes)):
        delay_spawn = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=prev_spawn, on_exit=[gz_spawn_nodes[i]])
        )
        actions.append(delay_spawn)
        prev_spawn = gz_spawn_nodes[i]

    delay_then_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=prev_spawn,
            on_exit=[TimerAction(period=8.0, actions=[joint_state_broadcasters[0]])],
        ),
    )
    actions.append(delay_then_joint_state)

    prev_action = joint_state_broadcasters[0]
    for i in range(1, len(joint_state_broadcasters)):
        after = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=prev_action, on_exit=[joint_state_broadcasters[i]])
        )
        actions.append(after)
        prev_action = joint_state_broadcasters[i]

    prev_action = joint_state_broadcasters[-1]
    for i in range(len(scaled_joint_trajectory_spawners)):
        after = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=prev_action, on_exit=[scaled_joint_trajectory_spawners[i]])
        )
        actions.append(after)
        prev_action = scaled_joint_trajectory_spawners[i]

    prev_action = scaled_joint_trajectory_spawners[-1]
    for i in range(len(forward_position_spawners)):
        after = RegisterEventHandler(
            event_handler=OnProcessExit(target_action=prev_action, on_exit=[forward_position_spawners[i]])
        )
        actions.append(after)
        prev_action = forward_position_spawners[i]

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
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
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start gazebo with GUI?",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
