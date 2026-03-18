import math
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


ROS_NAME_PATTERN = re.compile(r"^[a-zA-Z][a-zA-Z0-9_]*$")
FORBIDDEN_WORLD_CHARS = (";", "|", "&", "$", "`")


def _parse_robot_count(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise ValueError(f"robot_count must be an integer, got: {value!r}") from exc
    if parsed < 1 or parsed > 16:
        raise ValueError(f"robot_count must be in range [1, 16], got: {parsed}")
    return parsed


def _validate_ros_name(value: str, field_name: str) -> str:
    if not value or not ROS_NAME_PATTERN.match(value):
        raise ValueError(
            f"{field_name} must match [a-zA-Z][a-zA-Z0-9_]*, got: {value!r}"
        )
    return value


def _validate_world_file(world_file: str) -> str:
    if any(ch in world_file for ch in FORBIDDEN_WORLD_CHARS):
        raise ValueError(f"world_file contains forbidden shell characters: {world_file!r}")
    if ".." in world_file.replace("\\", "/").split("/"):
        raise ValueError(f"world_file must not contain path traversal '..': {world_file!r}")
    return world_file


def _parse_positions(positions_raw: str, robot_count: int):
    entries = [entry.strip() for entry in positions_raw.split(";") if entry.strip()]
    if len(entries) != robot_count:
        raise ValueError(
            "robot_positions must contain exactly "
            f"{robot_count} entries, got: {len(entries)}"
        )

    parsed = []
    for idx, entry in enumerate(entries, start=1):
        parts = [part.strip() for part in entry.split(",")]
        if len(parts) != 4:
            raise ValueError(
                f"robot_positions entry {idx} must be x,y,z,yaw (4 values), got: {entry!r}"
            )
        try:
            x, y, z, yaw = (float(part) for part in parts)
        except ValueError as exc:
            raise ValueError(
                f"robot_positions entry {idx} contains non-numeric values: {entry!r}"
            ) from exc
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            raise ValueError(
                f"robot_positions entry {idx} contains non-finite values: {entry!r}"
            )
        parsed.append((x, y, z, yaw))
    return parsed


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    safety_limits = LaunchConfiguration("safety_limits").perform(context)
    safety_pos_margin = LaunchConfiguration("safety_pos_margin").perform(context)
    safety_k_position = LaunchConfiguration("safety_k_position").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)
    world_file = _validate_world_file(LaunchConfiguration("world_file").perform(context))

    robot_count = _parse_robot_count(LaunchConfiguration("robot_count").perform(context))
    positions = _parse_positions(
        LaunchConfiguration("robot_positions").perform(context),
        robot_count,
    )
    namespace_prefix = _validate_ros_name(
        LaunchConfiguration("robot_namespace_prefix").perform(context),
        "robot_namespace_prefix",
    )
    launch_rviz_first_robot = (
        LaunchConfiguration("launch_rviz_first_robot").perform(context).lower() == "true"
    )

    moveit_launch_file = LaunchConfiguration("moveit_launch_file").perform(context)
    launch_servo = LaunchConfiguration("launch_servo").perform(context)
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    ).perform(context)
    warehouse_sqlite_base = LaunchConfiguration("warehouse_sqlite_base").perform(context)

    control_launch_path = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
    )

    actions = []
    for index, (x, y, z, yaw) in enumerate(positions):
        robot_namespace = f"{namespace_prefix}{index + 1}"
        _validate_ros_name(robot_namespace, "robot_namespace")

        launch_world = "true" if index == 0 else "false"
        launch_rviz = "true" if launch_rviz_first_robot and index == 0 else "false"
        warehouse_sqlite_path = f"{warehouse_sqlite_base}_{robot_namespace}.sqlite"

        robot_group = GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(control_launch_path),
                    launch_arguments={
                        "ur_type": ur_type,
                        "safety_limits": safety_limits,
                        "safety_pos_margin": safety_pos_margin,
                        "safety_k_position": safety_k_position,
                        "controllers_file": controllers_file,
                        "description_file": description_file,
                        "activate_joint_controller": activate_joint_controller,
                        "initial_joint_controller": initial_joint_controller,
                        "gazebo_gui": gazebo_gui,
                        "world_file": world_file,
                        "launch_gz_world": launch_world,
                        "launch_rviz": "false",
                        "robot_namespace": robot_namespace,
                        "robot_name": robot_namespace,
                        "robot_model_name": "ur",
                        "spawn_x": str(x),
                        "spawn_y": str(y),
                        "spawn_z": str(z),
                        "spawn_yaw": str(yaw),
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_launch_file),
                    launch_arguments={
                        "ur_type": ur_type,
                        "use_sim_time": "true",
                        "launch_rviz": launch_rviz,
                        "launch_servo": launch_servo,
                        "publish_robot_description_semantic": publish_robot_description_semantic,
                        "robot_namespace": robot_namespace,
                        "robot_description_topic": "robot_description",
                        "robot_model_name": "ur",
                        "warehouse_sqlite_path": warehouse_sqlite_path,
                    }.items(),
                ),
            ],
        )

        actions.append(robot_group)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
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
                description="Type/series of used UR robot.",
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
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
                ),
                description="Absolute path to YAML file with the controllers configuration.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
                ),
                description="URDF/XACRO description file (absolute path) with the robot.",
            ),
            DeclareLaunchArgument(
                "activate_joint_controller",
                default_value="true",
                description="Enable headless mode for robot control.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
                description="Robot controller to start.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", description="Start Gazebo with GUI?"
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.sdf",
                description="Gazebo world file (absolute path or world collection filename).",
            ),
            DeclareLaunchArgument(
                "robot_count",
                default_value="2",
                description="Number of UR robots to spawn (1..16).",
            ),
            DeclareLaunchArgument(
                "robot_positions",
                default_value="0,0,0,0;1.5,0,0,1.57",
                description="Semicolon-separated list of x,y,z,yaw(rad) poses.",
            ),
            DeclareLaunchArgument(
                "robot_namespace_prefix",
                default_value="ur",
                description="Prefix used to generate per-robot namespaces (ur1, ur2, ...).",
            ),
            DeclareLaunchArgument(
                "launch_rviz_first_robot",
                default_value="true",
                description="Launch RViz only for first robot namespace.",
            ),
            DeclareLaunchArgument(
                "launch_servo",
                default_value="false",
                description="Launch MoveIt Servo for each robot instance.",
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="MoveGroup publishes robot description semantic.",
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_base",
                default_value=os.path.expanduser("~/.ros/warehouse_ros"),
                description="Warehouse sqlite path prefix. Namespace suffix is auto-appended.",
            ),
            DeclareLaunchArgument(
                "moveit_launch_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
                ),
                description="Absolute path for MoveIt launch file.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
