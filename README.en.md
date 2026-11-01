🇵🇱 [Polski](README.md) | 🇬🇧 **English**

# 🤖 Cybuś — Emergency & Motion Nodes (ROS)

![ROS](https://img.shields.io/badge/ROS_Melodic-22314E?logo=ros&logoColor=white) ![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white) ![Docker](https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white) ![NVIDIA Jetson](https://img.shields.io/badge/NVIDIA_Jetson-76B900?logo=nvidia&logoColor=white)

> 🎓 Project in the **KN CyberTech** student research group (Wrocław University of Science and Technology)

Emergency nodes for **Cybuś** — the group's mobile robot built on the JetBot Pro platform (NVIDIA Jetson). Their job is safety: detecting obstacles with a lidar and interrupting the current movement immediately when something is in the way. Plus a motion node that turns commands like "drive 1 m forward" or "turn right 90°" into precise movement.

## ✨ Nodes

- **`cybus_motion_node`** — takes JSON commands (drive forward/backward, turn left/right, stop — command names are in Polish, e.g. `SKREC_W_PRAWO`) and executes them one at a time with **odometry feedback** until the robot has covered the distance or turned the angle; `ZATRZYMAJ_SIE` aborts the ongoing move and clears the queue, and a move with no odometry progress is aborted after a time limit
- **`emergency_stop_node`** — **emergency stop** triggered by the lidar (`/lidar/alert`): it publishes the state on `/emergency_stop`, and the motion node aborts the ongoing drive or turn right away instead of queuing the stop until the move ends (separate callback thread + a flag checked in the control loop). It is fail-safe: movement stays blocked until the lidar reports a clear path, and whenever the lidar has been silent for more than 1 s. During an e-stop only backing away is allowed (`JEDZ_DO_TYLU`, max 0.3 m)
- **`lidar_reader`** — reads the lidar in the **front sector** (120°, parameters `~sector_center_deg` / `~sector_width_deg`) and raises an alarm (`/lidar/alert`) when an obstacle is closer than **0.5 m**, with hysteresis (the alarm clears above 0.6 m). The rear is not covered by the lidar
- CSI camera driver for the Jetson

Detailed design assumptions for the e-stop and motion (parameters, limitations), in Polish: [ZALOZENIA.md](ZALOZENIA.md).

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

Example command (without a lidar, `/lidar/alert` has to be simulated continuously, otherwise the e-stop blocks movement):

```bash
rostopic pub -r 10 /lidar/alert std_msgs/Bool false &
rostopic pub -1 /motion_cmd std_msgs/String '{data: "{\"action\": \"SKREC_W_PRAWO\", \"value\": 90}"}'
```

## 🧪 Tests

Integration test of the e-stop, motion and lidar nodes in ROS Melodic, no robot needed (simulated odometry and lidar, ~1 min):

```bash
docker compose run --rm ros_melodic bash /root/catkin_ws/catkin_ws/src/cybus_robot/test/run_tests.sh
```

The test runs its own `roscore` on port 11411 (`ROS_TEST_PORT`), so it does not interfere with a ROS system running alongside. Exit code: 0 = all PASS, 1 = failures, 2 = environment problem (e.g. port in use). What is checked: [ZALOZENIA.md](ZALOZENIA.md#8-weryfikacja) (in Polish).
