# Copyright (c) 2025
# Dual UR Gazebo + MoveIt - two robots with planning in RViz

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    description_file = LaunchConfiguration("description_file")

    ur_control_two_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_two_robots.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_file": description_file,
            "launch_rviz": "false",
        }.items(),
    )

    ur_moveit_dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_dual_moveit_config"), "launch", "ur_moveit_dual.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_file": description_file,
            "launch_rviz": "true",
            "use_sim_time": "true",
        }.items(),
    )

    return [ur_control_two_launch, ur_moveit_dual_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e", "ur10e",
                "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30",
            ],
            default_value="ur5e",
        ),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"
            ]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
