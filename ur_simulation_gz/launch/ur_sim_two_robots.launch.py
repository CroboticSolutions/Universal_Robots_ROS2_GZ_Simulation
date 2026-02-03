# Copyright (c) 2025
# Two-robot Gazebo simulation launch file.
# Does not modify existing ur_sim_control / ur_sim_moveit - use those for single robot.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
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
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    pkg_share = FindPackageShare("ur_simulation_gz")
    controllers_robot1 = PathJoinSubstitution([pkg_share, "config", "ur_controllers_robot1.yaml"])
    controllers_robot2 = PathJoinSubstitution([pkg_share, "config", "ur_controllers_robot2.yaml"])

    # Robot 1 description: name=ur1, tf_prefix=robot1_, ros_namespace=robot1, origin 0 0 0
    robot1_description = Command(
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
            'tf_prefix:=robot1_',
            " ",
            "simulation_controllers:=",
            controllers_robot1,
            " ",
            "ros_namespace:=robot1",
            " ",
            'origin_xyz:="0 0 0"',
        ]
    )

    # Robot 2 description: name=ur2, tf_prefix=robot2_, ros_namespace=robot2, origin 1.5 0 0
    robot2_description = Command(
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
            'tf_prefix:=robot2_',
            " ",
            "simulation_controllers:=",
            controllers_robot2,
            " ",
            "ros_namespace:=robot2",
            " ",
            'origin_xyz:="1.5 0 0"',
        ]
    )

    # Gazebo
    gz_launch = IncludeLaunchDescription(
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

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Robot 1 state publisher
    robot1_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="robot1",
        output="both",
        parameters=[{"use_sim_time": True, "robot_description": robot1_description}],
    )

    # Robot 2 state publisher
    robot2_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="robot2",
        output="both",
        parameters=[{"use_sim_time": True, "robot_description": robot2_description}],
    )

    # Spawn robot1 in Gazebo
    create_robot1 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-string", robot1_description, "-name", "ur1", "-allow_renaming", "true"],
    )

    # Spawn robot2 in Gazebo (after robot1)
    create_robot2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot2_description,
            "-name", "ur2",
            "-allow_renaming", "true",
            "-x", "1.5", "-y", "0", "-z", "0",
        ],
    )

    # Robot 1 controllers (use_sim_time for Gazebo)
    robot1_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/robot1/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    robot1_scaled = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/robot1/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    robot1_forward_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/robot1/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": True}],
    )

    # Robot 2 controllers
    robot2_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/robot2/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    robot2_scaled = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/robot2/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    robot2_forward_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/robot2/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": True}],
    )

    # Chain: create_robot1 -> robot1 controllers -> create_robot2 -> robot2 controllers
    delay_robot1_ctrl = RegisterEventHandler(
        OnProcessExit(target_action=create_robot1, on_exit=[robot1_joint_state])
    )
    delay_robot1_scaled = RegisterEventHandler(
        OnProcessExit(target_action=robot1_joint_state, on_exit=[robot1_scaled])
    )
    delay_robot1_fwd = RegisterEventHandler(
        OnProcessExit(target_action=robot1_scaled, on_exit=[robot1_forward_pos])
    )
    delay_create_robot2 = RegisterEventHandler(
        OnProcessExit(target_action=robot1_forward_pos, on_exit=[create_robot2])
    )
    delay_robot2_ctrl = RegisterEventHandler(
        OnProcessExit(target_action=create_robot2, on_exit=[robot2_joint_state])
    )
    delay_robot2_scaled = RegisterEventHandler(
        OnProcessExit(target_action=robot2_joint_state, on_exit=[robot2_scaled])
    )
    delay_robot2_fwd = RegisterEventHandler(
        OnProcessExit(target_action=robot2_scaled, on_exit=[robot2_forward_pos])
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    )

    delay_rviz_after_robot2_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=robot2_joint_state, on_exit=[rviz_node]),
        condition=IfCondition(launch_rviz),
    )

    return [
        gz_launch,
        gz_bridge,
        robot1_state_pub,
        robot2_state_pub,
        create_robot1,
        delay_robot1_ctrl,
        delay_robot1_scaled,
        delay_robot1_fwd,
        delay_create_robot2,
        delay_robot2_ctrl,
        delay_robot2_scaled,
        delay_robot2_fwd,
        delay_rviz_after_robot2_joint_state,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("world_file", default_value="empty.sdf"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"), "config", "two_robots.rviz"
            ]),
            description="RViz config file for two-robot visualization.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"
            ]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
