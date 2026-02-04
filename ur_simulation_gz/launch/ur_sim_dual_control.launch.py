# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Dual-arm variant: 1 world, 2 robots (ur1, ur2) with namespaces.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_file = LaunchConfiguration("description_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    pkg_share = FindPackageShare("ur_simulation_gz")
    controllers_ur1 = PathJoinSubstitution([pkg_share, "config", "ur_controllers_ur1.yaml"])
    controllers_ur2 = PathJoinSubstitution([pkg_share, "config", "ur_controllers_ur2.yaml"])

    # Robot 1 (ur1) description
    robot_description_ur1 = Command(
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
            "name:=ur1",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=ur1_",
            " ",
            "ros_namespace:=ur1",
            " ",
            "simulation_controllers:=",
            controllers_ur1,
        ]
    )

    # Robot 2 (ur2) description
    robot_description_ur2 = Command(
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
            "name:=ur2",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=ur2_",
            " ",
            "ros_namespace:=ur2",
            " ",
            "simulation_controllers:=",
            controllers_ur2,
        ]
    )

    # Robot state publishers (one per namespace)
    robot_state_publisher_ur1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ur1",
        output="both",
        parameters=[{"use_sim_time": True}, {"robot_description": robot_description_ur1}],
    )
    robot_state_publisher_ur2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ur2",
        output="both",
        parameters=[{"use_sim_time": True}, {"robot_description": robot_description_ur2}],
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

    # Spawn ur1 (first robot, includes world/ground from xacro)
    gz_spawn_ur1 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_ur1,
            "-name",
            "ur1",
            "-allow_renaming",
            "true",
        ],
    )

    # Spawn ur2 (second robot, offset in x)
    gz_spawn_ur2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_ur2,
            "-name",
            "ur2",
            "-allow_renaming",
            "true",
            "-x",
            "1.0",
        ],
    )

    # Controller spawners for ur1
    joint_state_broadcaster_ur1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/ur1/controller_manager"],
    )
    scaled_joint_trajectory_ur1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/ur1/controller_manager"],
    )

    # Controller spawners for ur2
    joint_state_broadcaster_ur2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/ur2/controller_manager"],
    )
    scaled_joint_trajectory_ur2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/ur2/controller_manager"],
    )

    # Clock bridge
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Chain: spawn ur1 -> spawn ur2 -> spawn controllers for both
    delay_spawn_ur2 = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=gz_spawn_ur1, on_exit=[gz_spawn_ur2]),
    )
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_ur2,
            on_exit=[
                joint_state_broadcaster_ur1,
                joint_state_broadcaster_ur2,
                scaled_joint_trajectory_ur1,
                scaled_joint_trajectory_ur2,
            ],
        ),
    )

    return [
        gz_launch_description,
        gz_sim_bridge,
        robot_state_publisher_ur1,
        robot_state_publisher_ur2,
        gz_spawn_ur1,
        delay_spawn_ur2,
        delay_controllers,
    ]


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
