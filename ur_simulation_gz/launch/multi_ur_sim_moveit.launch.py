import math
import json
import os
import re
import time
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ROS_NAME_PATTERN = re.compile(r"^[a-zA-Z][a-zA-Z0-9_]*$")
FORBIDDEN_WORLD_CHARS = (";", "|", "&", "$", "`")
PROFILE_TO_FILE = {
    "default": "default.yaml",
    "lab": "lab.yaml",
    "lab_gripper": "lab_gripper.yaml",
    "lab_gripper_5": "lab_gripper_5.yaml",
    "lab_gripper_6": "lab_gripper_6.yaml",
    "lab_gripper_one": "lab_gripper_one.yaml",
    "stress10": "stress10.yaml",
}


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


def _parse_robot_count(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise ValueError(f"robot_count must be an integer, got: {value!r}") from exc
    if parsed < 1 or parsed > 16:
        raise ValueError(f"robot_count must be in range [1, 16], got: {parsed}")
    return parsed


def _parse_nonnegative_float(value: str, field_name: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"{field_name} must be a float, got: {value!r}") from exc
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{field_name} must be a non-negative finite float, got: {parsed!r}")
    return parsed


def _parse_optional_robot_count(value):
    if value is None:
        return None
    return _parse_robot_count(str(value))


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


def _validate_profile_yaml_path(path: str) -> str:
    """Absolute path to a generated multi-UR profile YAML (GUI / tooling)."""
    path = path.strip()
    if not path:
        raise ValueError("profile_yaml_path is empty")
    if any(ch in path for ch in FORBIDDEN_WORLD_CHARS):
        raise ValueError(f"profile_yaml_path contains forbidden characters: {path!r}")
    normalized = path.replace("\\", "/")
    if ".." in normalized.split("/"):
        raise ValueError(f"profile_yaml_path must not contain path traversal '..': {path!r}")
    if not os.path.isabs(path):
        raise ValueError(f"profile_yaml_path must be an absolute path, got: {path!r}")
    if not os.path.isfile(path):
        raise ValueError(f"profile_yaml_path file not found: {path!r}")
    return path


def _load_profile_from_path(absolute_path: str) -> dict:
    with open(absolute_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Profile YAML must be a mapping/object: {absolute_path}")
    return data


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


def _parse_profile_positions(positions_raw, robot_count: int):
    if not isinstance(positions_raw, list) or len(positions_raw) == 0:
        raise ValueError(
            "robot_positions in profile must be a non-empty list of entries "
            "with x, y, z, yaw fields."
        )
    if len(positions_raw) != robot_count:
        raise ValueError(
            "robot_positions in profile must contain exactly "
            f"{robot_count} entries, got: {len(positions_raw)}"
        )
    parsed = []
    for idx, entry in enumerate(positions_raw, start=1):
        if not isinstance(entry, dict):
            raise ValueError(
                f"robot_positions entry {idx} must be an object with x, y, z, yaw."
            )
        missing = [key for key in ("x", "y", "z", "yaw") if key not in entry]
        if missing:
            raise ValueError(
                f"robot_positions entry {idx} is missing required keys: {missing}"
            )
        try:
            x = float(entry["x"])
            y = float(entry["y"])
            z = float(entry["z"])
            yaw = float(entry["yaw"])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"robot_positions entry {idx} contains non-numeric values: {entry!r}"
            ) from exc
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            raise ValueError(
                f"robot_positions entry {idx} contains non-finite values: {entry!r}"
            )
        parsed.append((x, y, z, yaw))
    return parsed


def _parse_world_props(profile: dict) -> list[dict]:
    """Parse world_props from GUI profile YAML for ros_gz_sim create."""
    raw = profile.get("world_props")
    if not isinstance(raw, list) or len(raw) == 0:
        return []
    out = []
    for idx, entry in enumerate(raw, start=1):
        if not isinstance(entry, dict):
            raise ValueError(f"world_props entry {idx} must be an object")
        missing = [
            key
            for key in ("name", "model", "x", "y", "z", "roll", "pitch", "yaw")
            if key not in entry
        ]
        if missing:
            raise ValueError(f"world_props entry {idx} is missing keys: {missing}")
        name = str(entry["name"])
        model = str(entry["model"])
        _validate_ros_name(name, f"world_props[{idx}].name")
        _validate_ros_name(model, f"world_props[{idx}].model")
        try:
            x = float(entry["x"])
            y = float(entry["y"])
            z = float(entry["z"])
            roll = float(entry["roll"])
            pitch = float(entry["pitch"])
            yaw = float(entry["yaw"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"world_props entry {idx} contains non-numeric values: {entry!r}") from exc
        if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, yaw)):
            raise ValueError(f"world_props entry {idx} contains non-finite values: {entry!r}")
        out.append(
            {
                "name": name,
                "model": model,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        )
    return out


def _world_prop_sdf_string(name: str, model: str) -> str:
    """Single-line SDF wrapping model:// include for ros_gz_sim create -string."""
    is_coke_can = name == "coke_can" or model == "Coke"
    static_value = "false" if is_coke_can else "true"
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.10">'
        f'<model name="{name}"><static>{static_value}</static>'
        f'<include><uri>model://{model}</uri></include></model>'
        "</sdf>"
    )


def _matmul3(
    a: tuple[tuple[float, float, float], ...],
    b: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _rpy_to_rot_matrix(
    roll: float, pitch: float, yaw: float
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Rotation from model frame to world: v_world = R @ v_model (same ordering as SDF pose parsing)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = (
        (1.0, 0.0, 0.0),
        (0.0, cr, -sr),
        (0.0, sr, cr),
    )
    ry = (
        (cp, 0.0, sp),
        (0.0, 1.0, 0.0),
        (-sp, 0.0, cp),
    )
    rz = (
        (cy, -sy, 0.0),
        (sy, cy, 0.0),
        (0.0, 0.0, 1.0),
    )
    return _matmul3(_matmul3(rz, ry), rx)


def _mat_vec_mul(
    m: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z = v
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def _load_world_prop_centroid_offsets() -> dict[str, tuple[float, float, float]]:
    """Model name -> offset (m) from model origin to visual centroid; see world_prop_centroid_offset_m.yaml."""
    path = os.path.join(
        get_package_share_directory("ur_simulation_gz"),
        "config",
        "multi_ur",
        "world_prop_centroid_offset_m.yaml",
    )
    if not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    raw = data.get("centroid_offset_m")
    if not isinstance(raw, dict):
        return {}
    out: dict[str, tuple[float, float, float]] = {}
    for model, val in raw.items():
        if not isinstance(val, (list, tuple)) or len(val) != 3:
            continue
        ox, oy, oz = float(val[0]), float(val[1]), float(val[2])
        if all(math.isfinite(t) for t in (ox, oy, oz)):
            out[str(model)] = (ox, oy, oz)
    return out


def _world_prop_spawn_xyz_from_profile_centroid(
    centroid_offsets: dict[str, tuple[float, float, float]],
    model: str,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    """Map profile to ros_gz_sim create pose.

    Profile x,y = desired visual centroid in the horizontal plane (world XY).
    Profile z = model origin height in world (unchanged from pre-centroid behavior; no Z mesh fix).

    Spawn position = (profile_xy - R*(ox,oy,0), profile_z) with (ox,oy) from YAML (horizontal only).
    """
    ox, oy, _ = centroid_offsets.get(model, (0.0, 0.0, 0.0))
    r_mat = _rpy_to_rot_matrix(roll, pitch, yaw)
    rx, ry, _ = _mat_vec_mul(r_mat, (ox, oy, 0.0))
    return (x - rx, y - ry, z)


def _load_profile(profile_name: str):
    if profile_name not in PROFILE_TO_FILE:
        raise ValueError(
            "robots_profile must be one of "
            f"{sorted(PROFILE_TO_FILE.keys())}, got: {profile_name!r}"
        )
    profile_path = os.path.join(
        get_package_share_directory("ur_simulation_gz"),
        "config",
        "multi_ur",
        PROFILE_TO_FILE[profile_name],
    )
    if not os.path.isfile(profile_path):
        raise ValueError(f"Profile YAML file not found: {profile_path}")
    with open(profile_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Profile YAML must be a mapping/object: {profile_path}")
    return data


def launch_setup(context, *args, **kwargs):
    profile_yaml_path_raw = LaunchConfiguration("profile_yaml_path").perform(context).strip()
    if profile_yaml_path_raw:
        validated_path = _validate_profile_yaml_path(profile_yaml_path_raw)
        profile = _load_profile_from_path(validated_path)
        # For logging only; named-profile branch sets robots_profile from launch arg
        robots_profile = validated_path
    else:
        robots_profile = LaunchConfiguration("robots_profile").perform(context)
        profile = _load_profile(robots_profile)

    ur_type = str(profile.get("ur_type", LaunchConfiguration("ur_type").perform(context)))
    safety_limits = str(
        profile.get("safety_limits", LaunchConfiguration("safety_limits").perform(context))
    )
    safety_pos_margin = str(
        profile.get("safety_pos_margin", LaunchConfiguration("safety_pos_margin").perform(context))
    )
    safety_k_position = str(
        profile.get("safety_k_position", LaunchConfiguration("safety_k_position").perform(context))
    )
    controllers_file = str(
        profile.get("controllers_file", LaunchConfiguration("controllers_file").perform(context))
    )
    description_file = str(
        profile.get("description_file", LaunchConfiguration("description_file").perform(context))
    )
    use_robotiq_gripper_raw = str(
        profile.get(
            "use_robotiq_gripper",
            LaunchConfiguration("use_robotiq_gripper").perform(context),
        )
    )
    use_robotiq_gripper = "true" if use_robotiq_gripper_raw.lower() == "true" else "false"
    activate_joint_controller = str(
        profile.get(
            "activate_joint_controller",
            LaunchConfiguration("activate_joint_controller").perform(context),
        )
    )
    initial_joint_controller = str(
        profile.get(
            "initial_joint_controller",
            LaunchConfiguration("initial_joint_controller").perform(context),
        )
    )
    # Default launch arg is "inherit" so profile YAML can set GUI; explicit gazebo_gui:=true|false overrides profile.
    _gui_arg = LaunchConfiguration("gazebo_gui").perform(context)
    if _gui_arg != "inherit":
        gazebo_gui = _gui_arg
    else:
        gazebo_gui = str(profile.get("gazebo_gui", "true"))
    world_file = _validate_world_file(
        str(profile.get("world_file", LaunchConfiguration("world_file").perform(context)))
    )
    if not os.path.isabs(world_file) and world_file != "empty.sdf":
        pkg_worlds_dir = os.path.join(
            get_package_share_directory("ur_simulation_gz"), "worlds"
        )
        candidate = os.path.join(pkg_worlds_dir, world_file)
        if os.path.isfile(candidate):
            world_file = candidate
    gz_physics_engine = str(
        profile.get(
            "gz_physics_engine",
            LaunchConfiguration("gz_physics_engine").perform(context),
        )
    )
    namespace_prefix = _validate_ros_name(
        str(
            profile.get(
                "robot_namespace_prefix",
                LaunchConfiguration("robot_namespace_prefix").perform(context),
            )
        ),
        "robot_namespace_prefix",
    )
    launch_rviz_first_robot = str(
        profile.get(
            "launch_rviz_first_robot",
            LaunchConfiguration("launch_rviz_first_robot").perform(context),
        )
    ).lower() == "true"
    launch_servo = str(profile.get("launch_servo", LaunchConfiguration("launch_servo").perform(context)))
    publish_robot_description_semantic = str(
        profile.get(
            "publish_robot_description_semantic",
            LaunchConfiguration("publish_robot_description_semantic").perform(context),
        )
    )
    warehouse_sqlite_base = str(
        profile.get(
            "warehouse_sqlite_base",
            LaunchConfiguration("warehouse_sqlite_base").perform(context),
        )
    )
    moveit_launch_file = str(
        profile.get(
            "moveit_launch_file",
            LaunchConfiguration("moveit_launch_file").perform(context),
        )
    )
    semantic_description_file = str(
        profile.get(
            "semantic_description_file",
            LaunchConfiguration("semantic_description_file").perform(context),
        )
    )
    per_robot_start_delay_s = _parse_nonnegative_float(
        str(
            profile.get(
                "per_robot_start_delay_s",
                LaunchConfiguration("per_robot_start_delay_s").perform(context),
            )
        ),
        "per_robot_start_delay_s",
    )
    moveit_start_delay_s = _parse_nonnegative_float(
        str(
            profile.get(
                "moveit_start_delay_s",
                LaunchConfiguration("moveit_start_delay_s").perform(context),
            )
        ),
        "moveit_start_delay_s",
    )

    publish_moveit_table_collision = (
        str(profile.get("publish_moveit_table_collision", "false")).lower() == "true"
    )
    try:
        moveit_table_collision_extra_delay_s = float(
            profile.get("moveit_table_collision_extra_delay_s", 10.0)
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("moveit_table_collision_extra_delay_s must be a float") from exc
    if not math.isfinite(moveit_table_collision_extra_delay_s) or moveit_table_collision_extra_delay_s < 0.0:
        raise ValueError(
            f"moveit_table_collision_extra_delay_s must be non-negative finite, got: {moveit_table_collision_extra_delay_s!r}"
        )

    robot_count = _parse_optional_robot_count(profile.get("robot_count"))
    if robot_count is None:
        raw_positions = profile.get("robot_positions", [])
        robot_count = _parse_robot_count(str(len(raw_positions)))
    positions = _parse_profile_positions(profile.get("robot_positions"), robot_count)

    camera_gz_robots_raw = profile.get("camera_gz_robots", [])
    if not isinstance(camera_gz_robots_raw, list):
        camera_gz_robots_raw = []
    camera_gz_set = {
        str(x).strip().lower() for x in camera_gz_robots_raw if str(x).strip()
    }

    control_launch_path = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
    )

    # region agent log
    _agent_log(
        run_id="pre-fix",
        hypothesis_id="H1",
        location="multi_ur_sim_moveit.launch.py:launch_setup",
        message="profile resolved for multi-robot launch",
        data={
            "robots_profile": robots_profile,
            "robot_count": robot_count,
            "world_file": world_file,
            "gazebo_gui": gazebo_gui,
            "gz_physics_engine": gz_physics_engine,
            "controllers_file": controllers_file,
            "use_robotiq_gripper": use_robotiq_gripper,
            "per_robot_start_delay_s": per_robot_start_delay_s,
            "moveit_start_delay_s": moveit_start_delay_s,
            "camera_gz_set": sorted(camera_gz_set),
        },
    )
    # endregion

    actions = []
    for index, (x, y, z, yaw) in enumerate(positions):
        robot_namespace = f"{namespace_prefix}{index + 1}"
        _validate_ros_name(robot_namespace, "robot_namespace")

        launch_world = "true" if index == 0 else "false"
        launch_rviz = "true" if launch_rviz_first_robot and index == 0 else "false"
        warehouse_sqlite_path = f"{warehouse_sqlite_base}_{robot_namespace}.sqlite"

        camera_gz_enabled = "true" if robot_namespace.lower() in camera_gz_set else "false"

        # region agent log
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H3",
            location="multi_ur_sim_moveit.launch.py:robot_loop",
            message="per-robot include arguments prepared",
            data={
                "index": index,
                "robot_namespace": robot_namespace,
                "robot_name": robot_namespace,
                "spawn": {"x": x, "y": y, "z": z, "yaw": yaw},
                "launch_gz_world": launch_world,
                "camera_gz_enabled": camera_gz_enabled,
                "start_offset": float(index) * per_robot_start_delay_s,
            },
        )
        # endregion

        control_include = IncludeLaunchDescription(
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
                "use_robotiq_gripper": use_robotiq_gripper,
                "gz_physics_engine": gz_physics_engine,
                "camera_gz_enabled": camera_gz_enabled,
            }.items(),
        )
        moveit_include = IncludeLaunchDescription(
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
                "semantic_description_file": semantic_description_file,
            }.items(),
        )

        start_offset = float(index) * per_robot_start_delay_s
        robot_group = GroupAction(
            scoped=True,
            actions=[
                TimerAction(period=start_offset, actions=[control_include]),
                TimerAction(period=start_offset + moveit_start_delay_s, actions=[moveit_include]),
            ],
        )

        actions.append(robot_group)

    if publish_moveit_table_collision:
        for index in range(robot_count):
            robot_namespace = f"{namespace_prefix}{index + 1}"
            _validate_ros_name(robot_namespace, "robot_namespace")
            start_offset = float(index) * per_robot_start_delay_s
            table_timer_period = (
                start_offset + moveit_start_delay_s + moveit_table_collision_extra_delay_s
            )
            actions.append(
                TimerAction(
                    period=table_timer_period,
                    actions=[
                        Node(
                            package="ur_simulation_gz",
                            executable="publish_moveit_table_collision",
                            name="publish_moveit_table_collision",
                            namespace=robot_namespace,
                            parameters=[{"use_sim_time": True}],
                            output="screen",
                        )
                    ],
                )
            )

    world_props = _parse_world_props(profile)
    if world_props:
        gz_world_name = str(profile.get("gz_world_name", "lab_table_coke")).strip()
        _validate_ros_name(gz_world_name, "gz_world_name")
        centroid_offsets = _load_world_prop_centroid_offsets()
        try:
            spawn_delay = float(profile.get("world_props_spawn_delay_s", 8.0))
        except (TypeError, ValueError) as exc:
            raise ValueError("world_props_spawn_delay_s must be a float") from exc
        if not math.isfinite(spawn_delay) or spawn_delay < 0.0:
            raise ValueError(f"world_props_spawn_delay_s must be non-negative finite, got: {spawn_delay!r}")
        try:
            stagger = float(profile.get("world_props_stagger_s", 0.35))
        except (TypeError, ValueError) as exc:
            raise ValueError("world_props_stagger_s must be a float") from exc
        if not math.isfinite(stagger) or stagger < 0.0:
            raise ValueError(f"world_props_stagger_s must be non-negative finite, got: {stagger!r}")

        for index, prop in enumerate(world_props):
            sdf = _world_prop_sdf_string(prop["name"], prop["model"])
            period = spawn_delay + float(index) * stagger
            sx, sy, sz = _world_prop_spawn_xyz_from_profile_centroid(
                centroid_offsets,
                prop["model"],
                prop["x"],
                prop["y"],
                prop["z"],
                prop["roll"],
                prop["pitch"],
                prop["yaw"],
            )
            cmd = [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                gz_world_name,
                "-name",
                prop["name"],
                "-string",
                sdf,
                "-x",
                str(sx),
                "-y",
                str(sy),
                "-z",
                str(sz),
                "-R",
                str(prop["roll"]),
                "-P",
                str(prop["pitch"]),
                "-Y",
                str(prop["yaw"]),
                "-allow_renaming",
                "false",
            ]
            actions.append(
                TimerAction(
                    period=period,
                    actions=[
                        ExecuteProcess(
                            cmd=cmd,
                            output="screen",
                        )
                    ],
                )
            )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robots_profile",
                default_value="default",
                description="Multi-robot profile name. Supported: default, lab, lab_gripper, lab_gripper_one, lab_gripper_5, lab_gripper_6, stress10.",
            ),
            DeclareLaunchArgument(
                "profile_yaml_path",
                default_value="",
                description=(
                    "If non-empty, absolute path to a YAML file to use instead of robots_profile "
                    "(e.g. generated by arm_api2_gui). Empty uses robots_profile as before."
                ),
            ),
            DeclareLaunchArgument(
                "per_robot_start_delay_s",
                default_value="0.75",
                description="Delay in seconds between starting each robot stack.",
            ),
            DeclareLaunchArgument(
                "moveit_start_delay_s",
                default_value="2.0",
                description="Additional delay after control stack start before MoveIt launch.",
            ),
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
                "gazebo_gui",
                default_value="inherit",
                description=(
                    "Start Gazebo with GUI: true, false, or inherit (use gazebo_gui from robots_profile YAML)."
                ),
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.sdf",
                description="Gazebo world file (absolute path or world collection filename).",
            ),
            DeclareLaunchArgument(
                "gz_physics_engine",
                default_value="gz-physics-bullet-featherstone-plugin",
                description=(
                    "Passed to ur_sim_control / `gz sim --physics-engine`. "
                    "Bullet-Featherstone supports mimic constraints (Robotiq fingers)."
                ),
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
            DeclareLaunchArgument(
                "use_robotiq_gripper",
                default_value="false",
                choices=["true", "false"],
                description="Forwarded to ur_sim_control when not set in profile YAML.",
            ),
            DeclareLaunchArgument(
                "semantic_description_file",
                default_value="srdf/ur.srdf.xacro",
                description="MoveIt SRDF xacro path relative to ur_moveit_config (override via profile YAML).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
