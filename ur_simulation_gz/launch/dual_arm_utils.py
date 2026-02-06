# Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Shared utilities for dual-arm launch files.
# Apply joint name prefix to generic configs (robot_names argument).

import copy
import tempfile
from pathlib import Path

import yaml

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def apply_joint_prefix(data, prefix):
    """Recursively replace joint names in dict/list with prefixed versions."""
    if isinstance(data, dict):
        result = {}
        for k, v in data.items():
            if k in UR_JOINTS:
                result[prefix + k] = apply_joint_prefix(v, prefix)
            else:
                result[k] = apply_joint_prefix(v, prefix)
        return result
    if isinstance(data, list):
        return [
            (prefix + item) if item in UR_JOINTS else apply_joint_prefix(item, prefix)
            for item in data
        ]
    return data


def load_yaml_with_prefix(package_share, config_name, prefix):
    """Load YAML from package config, apply joint prefix, return transformed data."""
    path = Path(package_share) / "config" / config_name
    with open(path) as f:
        data = yaml.safe_load(f)
    return apply_joint_prefix(copy.deepcopy(data), prefix)


def write_prefixed_yaml_to_temp(data, suffix=".yaml"):
    """Write YAML data to a temp file, return path."""
    fd, path = tempfile.mkstemp(suffix=suffix)
    with open(fd, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    return path


def get_prefixed_controllers_path(package_share, prefix):
    """Load ur_controllers_sim.yaml, apply prefix (if non-empty), return path.
    With prefix='', returns path to generic config directly (no temp file).
    """
    if not prefix:
        return str(Path(package_share) / "config" / "ur_controllers_sim.yaml")
    data = load_yaml_with_prefix(package_share, "ur_controllers_sim.yaml", prefix)
    return write_prefixed_yaml_to_temp(data)


def get_prefixed_moveit_controllers_path(package_share, prefix):
    """Load moveit_controllers.yaml, apply prefix (if non-empty), return path.
    With prefix='', returns path to generic config directly (no temp file).
    """
    if not prefix:
        return str(Path(package_share) / "config" / "moveit_controllers.yaml")
    data = load_yaml_with_prefix(package_share, "moveit_controllers.yaml", prefix)
    return write_prefixed_yaml_to_temp(data)


def get_prefixed_joint_limits(package_share, prefix):
    """Load joint_limits.yaml, apply prefix (if non-empty), return joint_limits dict.
    With prefix='', returns generic joint_limits directly.
    """
    path = Path(package_share) / "config" / "joint_limits.yaml"
    with open(path) as f:
        data = yaml.safe_load(f)
    limits = data.get("joint_limits", data)
    if not prefix:
        return limits
    return apply_joint_prefix({"joint_limits": limits}, prefix).get("joint_limits", limits)
