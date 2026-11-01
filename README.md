🇵🇱 **Polski** | 🇬🇧 [English](README.en.md)

# 🤖 Cybuś — węzły awaryjne i ruchu (ROS)

![ROS](https://img.shields.io/badge/ROS_Melodic-22314E?logo=ros&logoColor=white) ![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white) ![Docker](https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white) ![NVIDIA Jetson](https://img.shields.io/badge/NVIDIA_Jetson-76B900?logo=nvidia&logoColor=white)

> 🎓 Projekt w kole naukowym **KN CyberTech** (Politechnika Wrocławska)

Węzły awaryjne dla **Cybusia** — robota mobilnego koła na platformie JetBot Pro (NVIDIA Jetson). Ich zadanie to bezpieczeństwo: wykrywanie przeszkód lidarem i natychmiastowe przerwanie bieżącego ruchu, gdy coś stanie na drodze. Do tego węzeł ruchu, który zamienia komendy typu „jedź 1 m do przodu” czy „skręć w prawo o 90°” na precyzyjny ruch.

## ✨ Węzły

- **`cybus_motion_node`** — przyjmuje komendy w JSON (`JEDZ_DO_PRZODU`, `JEDZ_DO_TYLU`, `SKREC_W_LEWO`, `SKREC_W_PRAWO`, `ZATRZYMAJ_SIE`) i wykonuje je po kolei z **kontrolą na podstawie odometrii**, aż robot przejedzie zadany dystans lub obróci się o zadany kąt; `ZATRZYMAJ_SIE` przerywa trwający ruch i czyści kolejkę, a ruch bez postępu odometrii jest przerywany po limicie czasu
- **`emergency_stop_node`** — **zatrzymanie awaryjne** wyzwalane przez lidar (`/lidar/alert`): publikuje stan na `/emergency_stop`, a węzeł ruchu przerywa trwającą jazdę lub obrót od razu, zamiast kolejkować stop do końca ruchu (osobny wątek callbacku + flaga sprawdzana w pętli sterowania). Działa w trybie fail-safe: ruch jest zablokowany, dopóki lidar nie potwierdzi wolnej drogi, oraz gdy lidar milczy dłużej niż 1 s. Podczas e-stopu dozwolone jest tylko cofnięcie (`JEDZ_DO_TYLU`, maks. 0,3 m), żeby odjechać od przeszkody
- **`lidar_reader`** — odczyt lidaru w **przednim sektorze** (120°, parametry `~sector_center_deg` / `~sector_width_deg`), alarm (`/lidar/alert`), gdy przeszkoda jest bliżej niż **0,5 m**, z histerezą (alarm gaśnie powyżej 0,6 m). Tył nie jest chroniony lidarem
- Sterownik kamery CSI dla Jetsona

Szczegółowe założenia działania e-stopu i ruchu (parametry, ograniczenia): [ZALOZENIA.md](ZALOZENIA.md).

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

Przykładowa komenda (bez lidaru trzeba symulować `/lidar/alert` w sposób ciągły, inaczej e-stop zablokuje ruch):

```bash
rostopic pub -r 10 /lidar/alert std_msgs/Bool false &
rostopic pub -1 /motion_cmd std_msgs/String '{data: "{\"action\": \"SKREC_W_PRAWO\", \"value\": 90}"}'
```

## 🧪 Testy

Test integracyjny e-stopu, ruchu i lidaru w ROS Melodic, bez robota (symulowana odometria i lidar, ok. 1 min):

```bash
docker compose run --rm ros_melodic bash /root/catkin_ws/catkin_ws/src/cybus_robot/test/run_tests.sh
```

Test uruchamia własny `roscore` na porcie 11411 (`ROS_TEST_PORT`), więc nie koliduje z ROS-em działającym obok. Kod wyjścia: 0 = wszystko PASS, 1 = błędy, 2 = problem ze środowiskiem (np. zajęty port). Co dokładnie jest sprawdzane: [ZALOZENIA.md](ZALOZENIA.md#8-weryfikacja).
