Universal_Robots_ROS2_GZ_Simulation
==========================================

Example files and configurations for Gazebo simulation of Universal Robots' manipulators.

## Build status
<table width="100%">
  <tr>
    <th></th>
    <th>Humble</th>
    <th>Jazzy</th>
    <th>Kilted</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/humble">humble</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2">ros2</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2">ros2</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2">ros2</a></td>
  </tr>
  <tr>
    <th>Build status</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/humble-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/humble-binary-main.yml/badge.svg?event=schedule"
              alt="Humble Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/jazzy-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/jazzy-binary-main.yml/badge.svg?event=schedule"
              alt="Jazzy Binary Main"/>
      </a> <br />
    </td>
    <td> <!-- Kilted -->
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/kilted-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/kilted-binary-main.yml/badge.svg?event=schedule"
              alt="Kilted Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/rolling-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/rolling-binary-main.yml/badge.svg?event=schedule"
              alt="Rolling Binary Main"/>
      </a> <br />
    </td>
  </tr>
</table>

A more [detailed build status](ci_status.md) shows the state of all CI workflows inside this repo.
Please note that the detailed view is intended for developers, while the one here should give end
users an overview of the current released state.


## Using the repository
Skip any of below steps is not applicable.

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspaces/ur_gz
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspaces/ur_gz` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ur_gz`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz
   rosdep update && rosdep install --ignore-src --from-paths src -y
   ```



### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install
  ```

## Running Executable
First, source your workspace

```
source $COLCON_WS/install/setup.bash
```

```
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

Move robot using test script from  `ur_robot_driver` package (if you've installed that one):
```
ros2 run ur_robot_driver example_move.py
```

Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```

## Multi-robot launch (profile-based)

You can launch multiple UR robots in one Gazebo world with namespaced control + MoveIt stacks.

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py
```

Select a profile explicitly:

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py \
  robots_profile:=lab
```

Stress profile (10 robots):

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py \
  robots_profile:=stress10
```

Four, five, or six robots with **Robotiq 2F-85** gripper (clone [PickNik `ros2_robotiq_gripper`](https://github.com/PickNikRobotics/ros2_robotiq_gripper) into the workspace as `robotiq_description` if the distro deb is unavailable):

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py \
  robots_profile:=lab_gripper
```

The `lab_gripper` profile uses world **`lab_table_coke.sdf`**: static **Table** and dynamic **Coke** (`model://Table`, `model://Coke` under `ur_simulation_gz/models/`). Override with `world_file:=empty.sdf` if you want a bare floor.

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py \
  robots_profile:=lab_gripper_5
```

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py \
  robots_profile:=lab_gripper_6
```

Single-robot Gazebo + gripper:

```
ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  use_robotiq_gripper:=true
```

Single-robot Gazebo + gripper + MoveIt (`srdf/ur_robotiq.srdf.xacro`):

```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py \
  use_robotiq_gripper:=true \
  semantic_description_file:=srdf/ur_robotiq.srdf.xacro
```

**MoveIt group `ur_manipulator`** ends at **`tool0`** (6 DOF) so planned trajectories match `scaled_joint_trajectory_controller`. Open/close the gripper via **`robotiq_gripper_controller`** (not part of that trajectory). TF still exposes the finger tip for IK/apps; use RViz **Planning Group** `ur_manipulator` and move the gripper with the separate gripper action when needed.

Gripper control uses `parallel_gripper_action_controller` (`robotiq_gripper_controller`), action `/<ns>/robotiq_gripper_controller/gripper_cmd`, message type `control_msgs/action/ParallelGripperCommand`. **arm_api2** sim config: `config/ur/ur_sim_robotiq.yaml` (parallel backend + finger tip `ee_link_name`).

`ur_sim_control.launch.py` always prepends `GZ_SIM_RESOURCE_PATH` / `IGN_GAZEBO_RESOURCE_PATH` with `share/ur_simulation_gz/models` so `model://Table` and `model://Coke` resolve. When `use_robotiq_gripper:=true`, it also prepends the parent of `robotiq_description/share` for `model://robotiq_description/...`. (RViz uses `package://` and does not need this.) **Table:** replace `models/Table/meshes/table.stl` with your own STL (same path) if the placeholder box mesh is not what you want.

**Physics engine (mimic / Robotiq):** By default the launch passes `gz sim --physics-engine gz-physics-bullet-featherstone-plugin` so URDF `<mimic>` constraints on the gripper are honored (Dartsim often rejects them with `gz_ros2_control`). Override with `gz_physics_engine:=gz-physics-dartsim-plugin` if you need the old engine. `ur_sim_moveit.launch.py` and `multi_ur_sim_moveit.launch.py` forward the same argument (or set optional `gz_physics_engine` in a multi-robot profile YAML).

### Lab table + Coke world

`worlds/lab_table_coke.sdf` — large ground plane, **static** table, **dynamic** Coke can (cylinder collision, mesh visual). Used by default in the **`lab_gripper`** multi-robot profile. The **Table** model uses **`meshes/stainless_steel_table.stl`** for the visual only; physics uses a single tabletop collision box (edit `models/Table/model.sdf` as needed).

### Grasp / friction smoke world

Optional world with a small dynamic cube and elevated ground friction:

```
ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  use_robotiq_gripper:=true \
  world_file:=$(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/worlds/grasp_smoke.sdf
```

Protocol (manual): pre-grasp → close gripper → lift 10 cm → hold. If the cube slips, raise `mu` on the cube/ground collisions or reduce cube mass; parallel-jaw + friction-only sims often need tuning.

### Multi-robot profile argument

- `robots_profile`: Multi-robot profile name. Supported values: `default`, `lab`, `lab_gripper`, `lab_gripper_5`, `lab_gripper_6`, `stress10`.
- Profiles are stored in `ur_simulation_gz/config/multi_ur/*.yaml`.
- Each profile YAML defines `robot_positions` as a list of objects with `x`, `y`, `z`, `yaw` (radians).
- Optional profile keys include: `ur_type`, `world_file`, `gazebo_gui`, `robot_namespace_prefix`, `launch_rviz_first_robot`, `use_robotiq_gripper` (`"true"` / `"false"`), `gz_physics_engine` (e.g. `gz-physics-bullet-featherstone-plugin` or `gz-physics-dartsim-plugin`), `semantic_description_file` (path relative to `ur_moveit_config` share, e.g. `srdf/ur_robotiq.srdf.xacro`).
- Launch supports startup staggering to reduce DDS/process spikes on large swarms:
  - `per_robot_start_delay_s`: delay between each robot stack start.
  - `moveit_start_delay_s`: extra delay before each robot's MoveIt stack starts.

### Common checks

Verify per-robot controllers are reachable:

```
ros2 control list_controllers -c /ur1/controller_manager
ros2 control list_controllers -c /ur2/controller_manager
```

Verify trajectory action endpoints:

```
ros2 action list | grep follow_joint_trajectory
```

### Troubleshooting

- **Profile not found**  
  Ensure `robots_profile` is one of the supported names (see Multi-robot profile argument).

- **`robot_positions` parse failure**  
  Ensure each profile entry has numeric `x`, `y`, `z`, `yaw` fields.

- **Namespace errors**  
  Namespace tokens must match `[a-zA-Z][a-zA-Z0-9_]*`.

- **Controller manager not found**  
  Use namespaced controller manager path: `-c /<namespace>/controller_manager`.

- **RViz not opening for all robots**  
  Default behavior opens RViz only for first robot (`launch_rviz_first_robot:=true`).
  Set `launch_rviz_first_robot:=false` for headless runs.
