import importlib.util
import os
import pytest

from ament_index_python.packages import get_package_share_directory


def _load_launch_module():
    launch_file = os.path.join(
        get_package_share_directory("ur_simulation_gz"),
        "launch",
        "multi_ur_sim_moveit.launch.py",
    )
    spec = importlib.util.spec_from_file_location("multi_ur_sim_moveit_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_parse_robot_count_valid():
    module = _load_launch_module()
    assert module._parse_robot_count("1") == 1
    assert module._parse_robot_count("16") == 16


@pytest.mark.parametrize("value", ["0", "17", "-1", "abc", "2.5"])
def test_parse_robot_count_invalid(value):
    module = _load_launch_module()
    with pytest.raises(ValueError):
        module._parse_robot_count(value)


def test_parse_positions_valid():
    module = _load_launch_module()
    positions = module._parse_positions("0,0,0,0;1.5,0,0,1.57", 2)
    assert positions == [(0.0, 0.0, 0.0, 0.0), (1.5, 0.0, 0.0, 1.57)]


@pytest.mark.parametrize(
    "positions,count",
    [
        ("0,0,0,0", 2),
        ("0,0,0;1,0,0,0", 2),
        ("0,0,0,a;1,0,0,0", 2),
    ],
)
def test_parse_positions_invalid(positions, count):
    module = _load_launch_module()
    with pytest.raises(ValueError):
        module._parse_positions(positions, count)


@pytest.mark.parametrize("name", ["ur1", "robot_A", "A1"])
def test_validate_ros_name_valid(name):
    module = _load_launch_module()
    assert module._validate_ros_name(name, "robot_namespace") == name


@pytest.mark.parametrize("name", ["", "1robot", "/ur1", "ur1/test", "~private", "ur-1"])
def test_validate_ros_name_invalid(name):
    module = _load_launch_module()
    with pytest.raises(ValueError):
        module._validate_ros_name(name, "robot_namespace")


@pytest.mark.parametrize("world_file", ["empty.sdf", "/tmp/world.sdf", "my_world.sdf"])
def test_validate_world_file_valid(world_file):
    module = _load_launch_module()
    assert module._validate_world_file(world_file) == world_file


@pytest.mark.parametrize("world_file", ["world;sdf", "../world.sdf", "foo|bar.sdf"])
def test_validate_world_file_invalid(world_file):
    module = _load_launch_module()
    with pytest.raises(ValueError):
        module._validate_world_file(world_file)


@pytest.mark.parametrize("profile_name", ["default", "lab", "stress10"])
def test_load_profile_valid(profile_name):
    module = _load_launch_module()
    profile = module._load_profile(profile_name)
    assert isinstance(profile, dict)
    assert "robot_positions" in profile


def test_load_profile_invalid():
    module = _load_launch_module()
    with pytest.raises(ValueError):
        module._load_profile("unknown_profile")


def test_parse_profile_positions_valid():
    module = _load_launch_module()
    profile_positions = [
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        {"x": 1.0, "y": 0.0, "z": 0.0, "yaw": 1.57},
    ]
    parsed = module._parse_profile_positions(profile_positions, 2)
    assert parsed == [(0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 1.57)]


def test_parse_profile_positions_invalid():
    module = _load_launch_module()
    with pytest.raises(ValueError):
        module._parse_profile_positions([{"x": 0.0, "y": 0.0}], 1)
