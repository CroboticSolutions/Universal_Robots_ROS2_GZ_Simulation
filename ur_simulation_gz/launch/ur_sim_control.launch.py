# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
# Author: Denis Stogl

import json
import time

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _agent_log(run_id: str, hypothesis_id: str, location: str, message: str, data: dict) -> None:
    payload = {
        "sessionId": "37d04d",
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    try:
        with open("/root/.cursor/debug-37d04d.log", "a", encoding="utf-8") as log_file:
            log_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
    except Exception:
        pass


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    launch_gz_world = LaunchConfiguration("launch_gz_world")
    gz_physics_engine = LaunchConfiguration("gz_physics_engine")
    use_robotiq_gripper = LaunchConfiguration("use_robotiq_gripper")
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_name = LaunchConfiguration("robot_name")
    robot_model_name = LaunchConfiguration("robot_model_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    camera_gz_enabled = LaunchConfiguration("camera_gz_enabled")
    camera_bridge_config_file = LaunchConfiguration("camera_bridge_config_file")

    namespace_value = robot_namespace.perform(context).strip("/")
    if namespace_value:
        controller_manager = f"/{namespace_value}/controller_manager"
    else:
        controller_manager = "/controller_manager"
    launch_gz_world_value = launch_gz_world.perform(context)
    gazebo_gui_value = gazebo_gui.perform(context)
    world_file_value = world_file.perform(context)
    gz_physics_engine_value = gz_physics_engine.perform(context)
    controllers_file_value = controllers_file.perform(context)
    use_robotiq_value = use_robotiq_gripper.perform(context)
    camera_gz_enabled_value = camera_gz_enabled.perform(context)

    # region agent log
    _agent_log(
        run_id="pre-fix",
        hypothesis_id="H2",
        location="ur_sim_control.launch.py:launch_setup",
        message="control launch resolved values",
        data={
            "namespace_value": namespace_value,
            "controller_manager": controller_manager,
            "launch_gz_world": launch_gz_world_value,
            "gazebo_gui": gazebo_gui_value,
            "world_file": world_file_value,
            "gz_physics_engine": gz_physics_engine_value,
            "controllers_file": controllers_file_value,
            "use_robotiq_gripper": use_robotiq_value,
            "camera_gz_enabled": camera_gz_enabled_value,
        },
    )
    # endregion

    robot_description_content = Command(
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
            "name:=",
            robot_model_name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "simulation_controllers:=",
            controllers_file,
            " ",
            "ros_namespace:=",
            robot_namespace,
            " ",
            "use_robotiq_gripper:=",
            use_robotiq_gripper,
            " ",
            "camera_gz_enabled:=",
            camera_gz_enabled,
        ]
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=namespace_value,
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        parameters=[
            {"use_sim_time": True},
            {
                "robot_description": ParameterValue(
                    robot_description_content,
                    value_type=str,
                )
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        namespace=namespace_value,
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace_value,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--param-file",
            controllers_file,
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace_value,
        arguments=[
            initial_joint_controller,
            "-c",
            controller_manager,
            "--controller-manager-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--param-file",
            controllers_file,
        ],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace_value,
        arguments=[
            initial_joint_controller,
            "-c",
            controller_manager,
            "--stopped",
            "--controller-manager-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--param-file",
            controllers_file,
        ],
        condition=UnlessCondition(activate_joint_controller),
    )
    delay_active_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[initial_joint_controller_spawner_started],
        ),
        condition=IfCondition(activate_joint_controller),
    )
    delay_inactive_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[initial_joint_controller_spawner_stopped],
        ),
        condition=UnlessCondition(activate_joint_controller),
    )

    # Spawn gripper only after the arm trajectory spawner finishes. Starting it in parallel with
    # the arm spawner (both on joint_state_broadcaster exit) is unreliable in multi-robot launch.
    robotiq_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace_value,
        arguments=[
            "robotiq_gripper_controller",
            "-c",
            controller_manager,
            "--controller-manager-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--param-file",
            controllers_file,
        ],
    )
    use_robotiq_flag = use_robotiq_gripper.perform(context).lower() == "true"
    activate_joint_flag = activate_joint_controller.perform(context).lower() == "true"
    gripper_after_arm_controller_handlers = []
    if use_robotiq_flag:
        if activate_joint_flag:
            gripper_after_arm_controller_handlers.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=initial_joint_controller_spawner_started,
                        on_exit=[robotiq_gripper_spawner],
                    ),
                )
            )
        else:
            gripper_after_arm_controller_handlers.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=initial_joint_controller_spawner_stopped,
                        on_exit=[robotiq_gripper_spawner],
                    ),
                )
            )

    # region agent log
    _agent_log(
        run_id="pre-fix",
        hypothesis_id="H4",
        location="ur_sim_control.launch.py:spawner_plan",
        message="spawner plan resolved",
        data={
            "namespace_value": namespace_value,
            "use_robotiq_flag": use_robotiq_flag,
            "activate_joint_flag": activate_joint_flag,
            "controller_manager": controller_manager,
            "initial_joint_controller": initial_joint_controller.perform(context),
        },
    )
    # endregion

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            robot_name,
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
            "-allow_renaming",
            "false",
        ],
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[
                    " -r -v 4 --physics-engine ",
                    gz_physics_engine,
                    " ",
                    world_file,
                ],
                else_value=[
                    " -s -r -v 4 --physics-engine ",
                    gz_physics_engine,
                    " ",
                    world_file,
                ],
            )
        }.items(),
        condition=IfCondition(launch_gz_world),
    )

    # Piper-style: one ros_gz_bridge YAML (clock + cameras + point clouds); see config/ur_gz_bridge.yaml
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": camera_bridge_config_file}],
        condition=IfCondition(launch_gz_world),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_active_controller_after_joint_state,
        delay_inactive_controller_after_joint_state,
        *gripper_after_arm_controller_handlers,
        gz_spawn_entity,
        gz_launch_description,
        gz_bridge,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
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
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
            description="Rviz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_physics_engine",
            default_value="gz-physics-bullet-featherstone-plugin",
            description=(
                "gz-physics engine plugin passed to `gz sim --physics-engine`. "
                "Bullet-Featherstone supports URDF mimic constraints (needed for Robotiq finger kinematics). "
                "Use gz-physics-dartsim-plugin for legacy DART-only behavior."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="ROS namespace for this robot instance (e.g. ur1). Empty string keeps root namespace.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur",
            description="Gazebo entity and robot name for this robot instance.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model_name",
            default_value="ur",
            description="Robot model name passed to xacro/MoveIt semantics.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn position X (m).")
    )
    declared_arguments.append(
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn position Y (m).")
    )
    declared_arguments.append(
        DeclareLaunchArgument("spawn_z", default_value="0.0", description="Spawn position Z (m).")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Spawn yaw rotation in radians.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_gz_world",
            default_value="true",
            description="Start Gazebo world and gz_bridge (clock+cameras from ur_gz_bridge.yaml). False for other robots in multi-UR.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_robotiq_gripper",
            default_value="false",
            choices=["true", "false"],
            description="If true, attach Robotiq 2F-85 (robotiq_description) and spawn robotiq_gripper_controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_gz_enabled",
            default_value="false",
            choices=["true", "false"],
            description="If true, spawn Gazebo rgbd_camera on camera_link (see ur_camera_macro).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_camera_bridge",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Deprecated: ignored. Single gz_bridge uses camera_bridge_config_file when Gazebo starts.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_bridge_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "config", "ur_gz_bridge.yaml"]
            ),
            description="ros_gz_bridge YAML: /clock + per-robot camera image/camera_info/points (Piper-style).",
        )
    )

    # model://Table, model://Coke live under share/ur_simulation_gz/models/
    simulation_models_path = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "models"]
    )
    # Gazebo resolves model://robotiq_description/... against GZ_SIM_RESOURCE_PATH.
    # Parent of share/robotiq_description is .../share so model URI maps to package meshes.
    robotiq_resource_parent = PathJoinSubstitution(
        [FindPackageShare("robotiq_description"), ".."]
    )
    gripper_env_condition = IfCondition(
        EqualsSubstitution(LaunchConfiguration("use_robotiq_gripper"), "true")
    )

    return LaunchDescription(
        declared_arguments
        + [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=simulation_models_path,
                prepend=True,
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=simulation_models_path,
                prepend=True,
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=robotiq_resource_parent,
                prepend=True,
                condition=gripper_env_condition,
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=robotiq_resource_parent,
                prepend=True,
                condition=gripper_env_condition,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
