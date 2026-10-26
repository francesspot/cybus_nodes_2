**Polski** | [English](README.en.md)

# Cybus — węzły ROS

Węzły ROS 1 (Melodic) dla **Cybusia** — robota mobilnego opartego na platformie JetBot Pro (NVIDIA Jetson). Węzły zamieniają tekstowe komendy wysokiego poziomu na precyzyjne ruchy, zapewniają zatrzymanie awaryjne i wykrywają przeszkody za pomocą lidaru.

## Pakiety

### `cybus_robot`

**`cybus_motion_node.py`** — wykonuje komendy ruchu z kontrolą na podstawie odometrii.

- Subskrybuje `/motion_cmd` (`std_msgs/String`, JSON) i `/odom_raw` (`nav_msgs/Odometry`).
- Publikuje `/cmd_vel` (`geometry_msgs/Twist`).
- Obsługiwane akcje:

| Akcja | `value` |
|-------|---------|
| `JEDZ_DO_PRZODU` | dystans w metrach |
| `JEDZ_DO_TYLU` | dystans w metrach |
| `SKREC_W_LEWO` | kąt w stopniach |
| `SKREC_W_PRAWO` | kąt w stopniach |
| `ZATRZYMAJ_SIE` | — |

Przykład:
```bash
rostopic pub -1 /motion_cmd std_msgs/String '{data: "{\"action\": \"SKREC_W_PRAWO\", \"value\": 90}"}'
```

Parametry: `~linear_speed` (0,3 m/s), `~angular_speed` (0,8 rad/s), `~dist_tolerance` (0,02 m), `~angle_tolerance` (1°).

**`emergency_stop_node.py`** — nasłuchuje na `/motion_cmd` komend zatrzymania (np. `stop`, `zatrzymaj_sie`, `cybuś stop`) i publikuje zerową prędkość z częstotliwością 10 Hz, dopóki nie przyjdzie komenda startu (np. `start`, `cybuś start`). Komendy mogą być w formacie JSON lub zwykłym tekstem.

**`launch/motion.launch`** — uruchamia bazę JetBot Pro (`jetbot_pro/jetbot.launch`) i węzeł ruchu.

### `lidar_threshold`

- **`lidar_reader.py`** — czyta `/scan` (`sensor_msgs/LaserScan`), publikuje odległość do najbliższej przeszkody na `/lidar/min_distance` oraz alarm na `/lidar/alert`, gdy coś jest bliżej niż **0,5 m**.
- **`alert_handler.py`** — zamienia `/lidar/alert` na komendy `STOP` / `GO` na `/robot/command`.
- **`launch/lidar_threshold.launch`** — uruchamia oba węzły.

### `jetson_csi_cam`

Sterownik kamery CSI dla Jetsona, dołączony jako submoduł gita. Pliki `camera_overrides.isp` i `Camera_overrides.tar.gz` zawierają ustawienia ISP kamery.

## Uruchomienie

### Na robocie

```bash
cd catkin_ws/catkin_ws
catkin_make
source devel/setup.bash

roslaunch cybus_robot motion.launch
roslaunch lidar_threshold lidar_threshold.launch
rosrun cybus_robot emergency_stop_node.py
```

`motion.launch` wymaga zainstalowanego pakietu `jetbot_pro` (sterownika bazy robota).

### W Dockerze (do pracy bez robota)

```bash
docker compose up -d
docker exec -it cybus_ros bash
cd /root/catkin_ws/catkin_ws && catkin_make
```

Kontener używa obrazu `ros:melodic-ros-base` i montuje `catkin_ws/` z repozytorium.
