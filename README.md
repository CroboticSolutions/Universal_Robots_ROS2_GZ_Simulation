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

## Multi-robot launch (parametric)

You can launch multiple UR robots in one Gazebo world with namespaced control + MoveIt stacks.

```
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py \
  robot_count:=2 \
  robot_positions:="0,0,0,0;1.5,0,0,1.57" \
  ur_type:=ur5e
```

### Multi-robot parameters

- `robot_count`: Number of robots (valid range `1..16`).
- `robot_positions`: Semicolon-separated robot poses in format `x,y,z,yaw` where `yaw` is in **radians**.
  - Example for 3 robots: `"0,0,0,0;1.5,0,0,1.57;3.0,0,0,0"`.
- `robot_namespace_prefix`: Prefix used for generated namespaces (`ur1`, `ur2`, ... by default).
- `launch_rviz_first_robot`: Launch RViz only for first robot namespace (default `true`).

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

- **`robot_positions` parse failure**  
  Ensure each robot entry has exactly 4 numeric values: `x,y,z,yaw`.

- **Count mismatch**  
  Number of entries in `robot_positions` must exactly match `robot_count`.

- **Namespace errors**  
  Namespace tokens must match `[a-zA-Z][a-zA-Z0-9_]*`.

- **Controller manager not found**  
  Use namespaced controller manager path: `-c /<namespace>/controller_manager`.

- **RViz not opening for all robots**  
  Default behavior opens RViz only for first robot (`launch_rviz_first_robot:=true`).
  Set `launch_rviz_first_robot:=false` for headless runs.
