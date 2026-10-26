🇵🇱 [Polski](README.md) | 🇬🇧 **English**

# 🤖 Cybuś — Emergency & Motion Nodes (ROS)

![ROS](https://img.shields.io/badge/ROS_Melodic-22314E?logo=ros&logoColor=white) ![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white) ![Docker](https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white) ![NVIDIA Jetson](https://img.shields.io/badge/NVIDIA_Jetson-76B900?logo=nvidia&logoColor=white)

> 🎓 Project in the **KN CyberTech** student research group (Wrocław University of Science and Technology)

Emergency nodes for **Cybuś** — the group's mobile robot built on the JetBot Pro platform (NVIDIA Jetson). Their job is safety: stopping the robot instantly on command and detecting obstacles with a lidar. Plus a motion node that turns commands like "drive 1 m forward" or "turn right 90°" into precise movement.

## ✨ Nodes

- **`cybus_motion_node`** — takes JSON commands (drive forward/backward, turn left/right, stop — command names are in Polish, e.g. `SKREC_W_PRAWO`) and executes them with **odometry feedback** until the robot has covered the distance or turned the angle
- **`emergency_stop_node`** — **emergency stop**: after a "stop" command (also "cybuś stop") it blocks movement until "start" arrives
- **`lidar_reader`** + **`alert_handler`** — reads the lidar, raises an alarm when an obstacle is closer than **0.5 m** and sends STOP / GO commands
- CSI camera driver for the Jetson

## 🛠️ Tech

ROS 1 Melodic · Python · rospy · Docker · NVIDIA Jetson · lidar

## 🚀 Run it

```bash
cd catkin_ws/catkin_ws
catkin_make && source devel/setup.bash
roslaunch cybus_robot motion.launch
roslaunch lidar_threshold lidar_threshold.launch
```

For working without the robot there's a Docker environment: `docker compose up -d` (image `ros:melodic-ros-base`).

Example command:

```bash
rostopic pub -1 /motion_cmd std_msgs/String '{data: "{\"action\": \"SKREC_W_PRAWO\", \"value\": 90}"}'
```
