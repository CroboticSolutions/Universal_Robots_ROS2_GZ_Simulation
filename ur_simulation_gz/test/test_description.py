# Copyright (c) 2023 FZI Forschungszentrum Informatik
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
# Author: Lukas Sackewitz

import os
import shutil
import subprocess
import tempfile
import pytest

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


@pytest.mark.parametrize(
    "ur_type",
    [
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
)
@pytest.mark.parametrize("prefix", ["", "my_ur_"])
def test_ur_urdf_xacro(ur_type, prefix):
    # Initialize Arguments
    safety_limits = "true"
    safety_pos_margin = "0.15"
    safety_k_position = "20"
    # General Arguments
    description_package = "ur_description"

    joint_limit_params = os.path.join(
        get_package_share_directory(description_package), "config", ur_type, "joint_limits.yaml"
    )
    kinematics_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "default_kinematics.yaml",
    )
    physical_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "physical_parameters.yaml",
    )
    visual_params = os.path.join(
        get_package_share_directory(description_package),
        "config",
        ur_type,
        "visual_parameters.yaml",
    )

    description_file_path = os.path.join(
        get_package_share_directory("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"
    )

    (_, tmp_urdf_output_file) = tempfile.mkstemp(suffix=".urdf")

    # Compose `xacro` and `check_urdf` command
    xacro_command = (
        f"{shutil.which('xacro')}"
        f" {description_file_path}"
        f" joint_limit_params:={joint_limit_params}"
        f" kinematics_params:={kinematics_params}"
        f" physical_params:={physical_params}"
        f" visual_params:={visual_params}"
        f" safety_limits:={safety_limits}"
        f" safety_pos_margin:={safety_pos_margin}"
        f" safety_k_position:={safety_k_position}"
        f" name:={ur_type}"
        f" prefix:={prefix}"
        f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    # Try to call processes but finally remove the temp file
    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert (
            check_urdf_process.returncode == 0
        ), "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description. Please check your xacro file."

    finally:
        os.remove(tmp_urdf_output_file)


def test_ur_gz_xacro_with_robotiq_gripper():
    """UR + Robotiq 2F-85 merge must expand to valid URDF when robotiq_description is present."""
    try:
        get_package_share_directory("robotiq_description")
    except PackageNotFoundError:
        pytest.skip("robotiq_description not installed (optional dependency for gripper)")

    description_file_path = os.path.join(
        get_package_share_directory("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"
    )
    controllers_path = os.path.join(
        get_package_share_directory("ur_simulation_gz"), "config", "ur_controllers.yaml"
    )
    (_, tmp_urdf_output_file) = tempfile.mkstemp(suffix=".urdf")
    xacro_command = (
        f"{shutil.which('xacro')}"
        f" {description_file_path}"
        f" ur_type:=ur5e"
        f" name:=ur"
        f" tf_prefix:=\"\""
        f" simulation_controllers:={controllers_path}"
        f" use_robotiq_gripper:=true"
        f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"
    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )
        assert xacro_process.returncode == 0, xacro_process.stderr.decode()
        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )
        assert check_urdf_process.returncode == 0, check_urdf_process.stderr.decode()
        with open(tmp_urdf_output_file, encoding="utf-8") as handle:
            urdf_text = handle.read()
        assert "robotiq_85_left_knuckle_joint" in urdf_text
    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == "__main__":
    test_ur_urdf_xacro()
