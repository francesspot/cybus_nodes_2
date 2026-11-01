# Założenia: zatrzymanie awaryjne i ruch

Opis tego, jak mają działać `lidar_reader`, `emergency_stop_node` i `cybus_motion_node`. Przy zmianach w tych węzłach ten plik jest punktem odniesienia: jeśli zachowanie się zmienia, zmienia się też tutaj.

## Przepływ

```
/scan ──► lidar_reader ──► /lidar/alert ──► emergency_stop_node ──► /emergency_stop (latched)
                                                     │                        │
                                                     └──► /cmd_vel (zero)     ▼
/motion_cmd (JSON) ──────────────────────────────────────────────────► cybus_motion_node ──► /cmd_vel
/odom_raw ───────────────────────────────────────────────────────────►
```

## 1. Wyzwalacz e-stopu

- Zatrzymanie awaryjne wyzwala **wyłącznie** `/lidar/alert` (`std_msgs/Bool`, `True` = przeszkoda). Komendy tekstowe („stop”, „cybuś stop”, „start”) nie mają na nie wpływu.
- `ZATRZYMAJ_SIE` na `/motion_cmd` to zwykły stop węzła ruchu, nie e-stop.

## 2. Lidar (`lidar_reader`)

- Liczy się tylko **przedni sektor** 120° (`~sector_center_deg` = 0, `~sector_width_deg` = 120). Przeszkody z boku poza sektorem i z tyłu nie wyzwalają alarmu.
- 0° to oś X ramki lasera. **Na robocie trzeba sprawdzić, czy to rzeczywiście przód** — przy lidarze zamontowanym obróconym (częste przy RPLidarze) ustawić `~sector_center_deg` na `180`.
- Histereza: alarm włącza się poniżej 0,5 m (`~threshold`), gaśnie dopiero powyżej 0,6 m (`~release_threshold`).
- Odczyty wg REP 117: `+inf` = nic w zasięgu (wolne), `-inf` = bliżej niż `range_min` (przeszkoda), `NaN` i wartości ≤ 0,01 m są pomijane.
- Stan publikowany jest przy każdym skanie. Gdy w sektorze nie ma żadnego użytecznego odczytu, węzeł **nic nie publikuje** — e-stop potraktuje to jak brak lidaru.

## 3. Węzeł e-stopu (`emergency_stop_node`)

- **Fail-safe na starcie:** stan początkowy to e-stop aktywny; ruch odblokowuje dopiero pierwsze `False` z lidaru.
- Reaguje tylko na **zmianę** stanu; `/emergency_stop` jest publikowany przy zmianie i jako latched.
- Przy włączeniu e-stopu publikuje **jeden** zerowy `Twist` na `/cmd_vel` (nie w pętli — ciągłe zera blokowałyby cofanie od przeszkody).
- **Watchdog:** gdy `/lidar/alert` milczy dłużej niż `~alert_timeout` (1 s), e-stop się włącza; wyłącza go dopiero świeże `False`.

## 4. Węzeł ruchu (`cybus_motion_node`)

- Komendy JSON: `JEDZ_DO_PRZODU` / `JEDZ_DO_TYLU` [m], `SKREC_W_LEWO` / `SKREC_W_PRAWO` [stopnie], `ZATRZYMAJ_SIE`.
- Ruchy wykonuje **po kolei jeden wątek roboczy**; callback `/motion_cmd` tylko waliduje i kolejkuje. Ruchy nigdy nie wykonują się równolegle.
- **E-stop przerywa trwający ruch od razu** (callback `/emergency_stop` działa w osobnym wątku, pętla sterowania sprawdza flagę w każdej iteracji, 20 Hz): ruch kończy się najpóźniej po ok. 1/20 s.
- Sprawdzenie e-stopu i publikacja prędkości są atomowe — po zerze z e-stopu nie wychodzi już żadna niezerowa prędkość. Krótki impuls alertu (True→False szybciej niż iteracja pętli) też przerywa ruch.
- Przerwany ruch **nie jest wznawiany** po odblokowaniu; komendy z kolejki przyjęte przed e-stopem są porzucane.
- Podczas e-stopu komendy ruchu są **odrzucane**, z jednym wyjątkiem: `JEDZ_DO_TYLU` skrócone do `~max_escape_distance` (0,3 m), żeby robot mógł odjechać od przeszkody z przodu. Takiego cofania e-stop nie przerywa (przerywa je `ZATRZYMAJ_SIE`).
- Ruch jest zablokowany także, gdy **węzeł e-stopu nie działa** (brak publishera `/emergency_stop`) — na starcie i po jego awarii.
- `ZATRZYMAJ_SIE` (i nieznana akcja) przerywa trwający ruch od razu i czyści kolejkę.
- Ruch jest przerywany, gdy odometria milczy dłużej niż `~odom_timeout` (0,5 s) albo gdy trwa dłużej niż `~timeout_factor` × czas nominalny + 2 s (np. poślizg kół).
- Obrót liczony jest bez zawijania kąta — obrót o więcej niż 180° jest wykonywany w zadanym kierunku.
- Nieprawidłowe komendy (zły JSON, `action` niebędące tekstem, `value` nie-liczba / NaN / inf) są logowane i ignorowane; węzeł działa dalej.
- `/motion_cmd` jest subskrybowany dopiero po pierwszej wiadomości z `/odom_raw`.

## 5. Środowisko

- ROS 1 Melodic, Python 2.7 (shebang `python`); kod zgodny także z Pythonem 3.
- Uruchomienie: `motion.launch` (ruch + e-stop) i `lidar_threshold.launch` (lidar) — oba są potrzebne, bez lidaru robot stoi.
- Bez lidaru (Docker, testy) `/lidar/alert` trzeba publikować w sposób ciągły, np. `rostopic pub -r 10 /lidar/alert std_msgs/Bool false`. Pojedyncze `rostopic pub -1` po sekundzie włączy watchdog.

## 6. Parametry

| Węzeł | Parametr | Domyślnie | Znaczenie |
|---|---|---|---|
| `lidar_reader` | `~threshold` | 0.5 | włączenie alarmu [m] |
| | `~release_threshold` | 0.6 | wyłączenie alarmu [m] |
| | `~sector_center_deg` | 0.0 | środek sektora w ramce lasera [°] |
| | `~sector_width_deg` | 120.0 | szerokość sektora [°] |
| `cybus_emergency` | `~alert_timeout` | 1.0 | cisza lidaru do włączenia e-stopu [s] |
| `cybus_motion` | `~linear_speed` | 0.3 | prędkość liniowa [m/s] |
| | `~angular_speed` | 0.8 | prędkość kątowa [rad/s] |
| | `~dist_tolerance` | 0.02 | tolerancja dystansu [m] |
| | `~angle_tolerance` | 1.0 | tolerancja kąta [°] |
| | `~max_escape_distance` | 0.3 | maks. cofnięcie podczas e-stopu [m] |
| | `~odom_timeout` | 0.5 | cisza odometrii do przerwania ruchu [s] |
| | `~timeout_factor` | 2.0 | mnożnik limitu czasu ruchu |

## 7. Świadome ograniczenia

- **Tył nie jest chroniony lidarem** — cofanie podczas e-stopu (do 0,3 m) odbywa się na ślepo, także gdy e-stop włączył się z powodu ciszy lidaru.
- E-stop nie blokuje **innych źródeł** `/cmd_vel` (np. teleop) — wysyła jedno zero, a ruch blokuje `cybus_motion_node`.
- `alert_handler.py` (publikuje `/robot/command`) jest w repo, ale nie jest uruchamiany ani używany.

## 8. Weryfikacja

Założenia z tego pliku sprawdza test integracyjny `catkin_ws/catkin_ws/src/cybus_robot/test/`:

```bash
docker compose run --rm ros_melodic bash /root/catkin_ws/catkin_ws/src/cybus_robot/test/run_tests.sh
```

`run_tests.sh` sprawdza końce linii (LF), składnię (Python 2) i pliki launch — przy błędzie kończy się od razu. Potem uruchamia własny `roscore` na osobnym porcie (`ROS_TEST_PORT`, domyślnie 11411; jeśli jest zajęty, test się nie uruchamia), trzy węzły przez `rosrun` (czyli sprawdza też shebangi — węzeł, który nie wstanie, jest zgłaszany z końcówką swojego logu) i `test_estop_motion.py`, który symuluje odometrię i lidar. Na koniec szuka wyjątków w logach węzłów i zatrzymuje tylko procesy, które sam uruchomił. Kod wyjścia: 0 = OK, 1 = błędy, 2 = problem ze środowiskiem.

| Scenariusz | Założenie |
|---|---|
| S0 | start fail-safe: e-stop aktywny i komendy odrzucane do pierwszego `False` z lidaru (3) |
| S1, S2 | jazda kończy się na dystansie; obrót o 270° bez zawijania kąta (4) |
| S3 | e-stop przerywa jazdę, jedno `True`, jedno zero zamiast pętli, odrzucanie komend, cofnięcie do 0,3 m, brak wznowienia, przyjęcie komendy po odblokowaniu (3, 4) |
| S4 | `ZATRZYMAJ_SIE` przerywa ruch i czyści kolejkę (4) |
| S5 | krótki impuls alertu też przerywa ruch (4) |
| S6 | watchdog: cisza lidaru > 1 s włącza e-stop (3) |
| S7, S8 | brak i zastój odometrii przerywają ruch (4) |
| S9 | nieprawidłowe komendy nie wywracają węzła (4) |
| S10 | awaria węzła e-stopu blokuje ruch (4) |
| S11 | `lidar_reader`: sektor przedni, histereza, `inf` / `-inf` / `NaN` (2) |

Ostatni przebieg: 2026-11-01, `ros:melodic-ros-base`, 32/32 PASS (przerwanie ruchu ok. 30–50 ms po e-stopie). Test nie uruchamia `motion.launch` w całości (brak pakietu `jetbot_pro` w obrazie) ani nie sprawdza prawdziwego lidaru — orientację sektora trzeba zweryfikować na robocie.
