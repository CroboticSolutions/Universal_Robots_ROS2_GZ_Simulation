# Simulated RGB-D camera (Gazebo) + `arm_api2_gui`

## Simulation (`ur_simulation_gz`)

1. **Profile** — In `config/multi_ur/<profile>.yaml` set:
   - `camera_gz_robots: [ur1, ur2]` — only these namespaces get a Gazebo `rgbd_camera` (sensor name `camera`, Piper-style URDF) and bridged ROS topics.
2. **Bridge (Piper-style)** — Single `ros_gz_bridge` `parameter_bridge` node (`gz_bridge`) loads **`config/ur_gz_bridge.yaml`**: `/clock` plus per-robot `image_raw`, `camera_info`, and `points` (PointCloud2). It starts when the first stack launches Gazebo (`launch_gz_world:=true`).
3. **World name** — YAML uses gz topics under `/world/lab_table_coke/...`. If you change the world file, update `<world name="...">` in that SDF and the `gz_topic_name` entries to match `gz topic -l`.
4. **ROS topics** (after bridge):
   - `/clock`
   - `/ur1/camera/image_raw`, `/ur1/camera/camera_info`, `/ur1/camera/points`
   - `/ur2/camera/image_raw`, `/ur2/camera/camera_info`, `/ur2/camera/points`
5. **Verification**
   ```bash
   gz topic -l | grep -E 'image|camera|clock|points'
   ros2 topic list | grep -E 'camera|clock'
   ros2 topic hz /ur1/camera/image_raw
   ```

## GUI (`arm_api2_gui`)

- **`config/ros2_config.json`**
  - `cameraGzRobots`: must list the same namespaces as `camera_gz_robots` (or leave empty to skip the “no camera” guard).
  - `cameraImageTopicSuffix`: default `camera/image_raw` (resolved as `/<namespace>/camera/image_raw`).
- **WebRTC** (`server_ros.py` or your bridge on ports **8081** / **8082**) must subscribe to the topic the browser sends in the offer body (`ros_image_topic`), e.g. `/ur1/camera/image_raw` when `ur1` is selected.

## Changing which robots have cameras

1. Edit `camera_gz_robots` in the profile YAML.
2. Edit `config/ur_gz_bridge.yaml` so `gz_topic_name` matches each spawned model name (`-name` in spawn = `ur1`, `ur2`, …).
3. Edit `cameraGzRobots` in `arm_api2_gui/config/ros2_config.json` to match.
