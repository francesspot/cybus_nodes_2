🇵🇱 **Polski** | 🇬🇧 [English](README.en.md)

# 🤖 Cybuś — węzły ROS dla robota mobilnego

![ROS](https://img.shields.io/badge/ROS_Melodic-22314E?logo=ros&logoColor=white) ![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white) ![Docker](https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white) ![NVIDIA Jetson](https://img.shields.io/badge/NVIDIA_Jetson-76B900?logo=nvidia&logoColor=white)

Oprogramowanie dla **Cybusia** — robota mobilnego na platformie JetBot Pro (NVIDIA Jetson). 🛞 Węzły zamieniają proste komendy typu „jedź 1 m do przodu” czy „skręć w prawo o 90°” na precyzyjny ruch, pilnują bezpieczeństwa i wykrywają przeszkody lidarem.

## ✨ Węzły

- 🧭 **`cybus_motion_node`** — przyjmuje komendy w JSON (`JEDZ_DO_PRZODU`, `JEDZ_DO_TYLU`, `SKREC_W_LEWO`, `SKREC_W_PRAWO`, `ZATRZYMAJ_SIE`) i wykonuje je z **kontrolą na podstawie odometrii**, aż robot przejedzie zadany dystans lub obróci się o zadany kąt
- 🛑 **`emergency_stop_node`** — **zatrzymanie awaryjne**: po komendzie „stop” (także „cybuś stop”) blokuje ruch, dopóki nie przyjdzie „start”
- 📡 **`lidar_reader`** + **`alert_handler`** — odczyt lidaru, alarm, gdy przeszkoda jest bliżej niż **0,5 m**, i komendy STOP / GO
- 📷 Sterownik kamery CSI dla Jetsona

## 🛠️ Technologie

ROS 1 Melodic · Python · rospy · Docker · NVIDIA Jetson · lidar

## 🚀 Uruchomienie

```bash
cd catkin_ws/catkin_ws
catkin_make && source devel/setup.bash
roslaunch cybus_robot motion.launch
roslaunch lidar_threshold lidar_threshold.launch
```

Do pracy bez robota jest środowisko w Dockerze: `docker compose up -d` (obraz `ros:melodic-ros-base`).

Przykładowa komenda:

```bash
rostopic pub -1 /motion_cmd std_msgs/String '{data: "{\"action\": \"SKREC_W_PRAWO\", \"value\": 90}"}'
```
