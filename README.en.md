[Polski](README.md) | **English**

# Cybus — ROS Nodes

ROS 1 (Melodic) nodes for **Cybus**, a mobile robot based on the JetBot Pro platform (NVIDIA Jetson). The nodes turn high-level text commands into precise movements, provide an emergency stop and detect obstacles with a lidar.

## Packages

### `cybus_robot`

**`cybus_motion_node.py`** — executes movement commands using odometry feedback.

- Subscribes to `/motion_cmd` (`std_msgs/String`, JSON) and `/odom_raw` (`nav_msgs/Odometry`).
- Publishes `/cmd_vel` (`geometry_msgs/Twist`).
- Supported actions:

| Action | `value` |
|--------|---------|
| `JEDZ_DO_PRZODU` (drive forward) | distance in metres |
| `JEDZ_DO_TYLU` (drive backward) | distance in metres |
| `SKREC_W_LEWO` (turn left) | angle in degrees |
| `SKREC_W_PRAWO` (turn right) | angle in degrees |
| `ZATRZYMAJ_SIE` (stop) | — |

Example:
```bash
rostopic pub -1 /motion_cmd std_msgs/String '{data: "{\"action\": \"SKREC_W_PRAWO\", \"value\": 90}"}'
```

Parameters: `~linear_speed` (0.3 m/s), `~angular_speed` (0.8 rad/s), `~dist_tolerance` (0.02 m), `~angle_tolerance` (1°).

**`emergency_stop_node.py`** — listens on `/motion_cmd` for stop commands (e.g. `stop`, `zatrzymaj_sie`, `cybuś stop`) and keeps publishing zero velocity at 10 Hz until a start command (e.g. `start`, `cybuś start`) arrives. Commands can be JSON or plain text.

**`launch/motion.launch`** — starts the JetBot Pro base (`jetbot_pro/jetbot.launch`) and the motion node.

### `lidar_threshold`

- **`lidar_reader.py`** — reads `/scan` (`sensor_msgs/LaserScan`), publishes the nearest obstacle distance on `/lidar/min_distance` and an alert on `/lidar/alert` when something is closer than **0.5 m**.
- **`alert_handler.py`** — turns `/lidar/alert` into `STOP` / `GO` commands on `/robot/command`.
- **`launch/lidar_threshold.launch`** — starts both nodes.

### `jetson_csi_cam`

Driver for the Jetson CSI camera, linked as a git submodule. `camera_overrides.isp` and `Camera_overrides.tar.gz` contain the camera ISP tuning.

## Running

### On the robot

```bash
cd catkin_ws/catkin_ws
catkin_make
source devel/setup.bash

roslaunch cybus_robot motion.launch
roslaunch lidar_threshold lidar_threshold.launch
rosrun cybus_robot emergency_stop_node.py
```

`motion.launch` needs the `jetbot_pro` package (the robot's base driver) to be installed.

### In Docker (for development without the robot)

```bash
docker compose up -d
docker exec -it cybus_ros bash
cd /root/catkin_ws/catkin_ws && catkin_make
```

The container uses the `ros:melodic-ros-base` image and mounts `catkin_ws/` from the repository.
