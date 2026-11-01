#!/usr/bin/env bash
# Test integracyjny e-stopu i ruchu w ROS Melodic, bez robota.
# Uruchomienie z katalogu głównego repo:
#   docker compose run --rm ros_melodic bash /root/catkin_ws/catkin_ws/src/cybus_robot/test/run_tests.sh
#
# Test ma własny roscore na osobnym porcie (ROS_TEST_PORT, domyślnie 11411), więc nie
# dotyka ROS-a działającego obok (np. na robocie, port 11311). Gdy na tym porcie już coś
# działa, test się nie uruchamia. Na koniec zatrzymuje tylko procesy, które sam uruchomił.
#
# Logi: $LOGS (domyślnie nowy katalog /tmp/cybus_test.XXXXXX; podany katalog nie jest czyszczony).
# Kod wyjścia: 0 = wszystko OK, 1 = błędy testu, 2 = problem ze środowiskiem.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(cd "$HERE/../.." && pwd)"
PORT="${ROS_TEST_PORT:-11411}"
LOGS="${LOGS:-$(mktemp -d /tmp/cybus_test.XXXXXX)}"
mkdir -p "$LOGS"

source /opt/ros/melodic/setup.bash
export ROS_PACKAGE_PATH="$SRC:$ROS_PACKAGE_PATH"
export ROS_MASTER_URI="http://localhost:$PORT"
export ROS_LOG_DIR="$LOGS/ros"

SCRIPTS=("$SRC"/cybus_robot/scripts/*.py "$SRC"/lidar_threshold/scripts/lidar_reader.py "$HERE"/*.py)
LAUNCHES=("$SRC"/cybus_robot/launch/motion.launch "$SRC"/lidar_threshold/launch/lidar_threshold.launch)
NODE_NAMES=(cybus_emergency cybus_motion lidar_reader)
NODE_PIDS=()
ROSCORE_PID=""
TEST_PID=""
fail=0

stop_pids() {  # sygnał pid... — po 5 s dobija KILL-em (rospy przechwytuje TERM i nie zawsze kończy proces)
  local sig="$1"; shift
  local pid alive
  for pid in "$@"; do kill -"$sig" "$pid" 2> /dev/null; done
  for _ in $(seq 25); do
    alive=0
    for pid in "$@"; do kill -0 "$pid" 2> /dev/null && alive=1; done
    [ "$alive" -eq 0 ] && return
    sleep 0.2
  done
  for pid in "$@"; do kill -KILL "$pid" 2> /dev/null; done
}

cleanup() {
  trap - INT TERM
  # Najpierw tester, żeby nie raportował FAIL-i z powodu zatrzymywanych węzłów
  [ -n "$TEST_PID" ] && stop_pids TERM "$TEST_PID"
  stop_pids TERM "${NODE_PIDS[@]}"
  [ -n "$ROSCORE_PID" ] && stop_pids INT "$ROSCORE_PID"
  wait 2> /dev/null
}
trap cleanup EXIT
trap 'echo "== Przerwano"; exit 130' INT TERM

# ------------------------------------------------------------------
echo "== Końce linii (LF)"
for f in "${SCRIPTS[@]}" "${LAUNCHES[@]}" "$HERE/run_tests.sh"; do
  if grep -q $'\r' "$f"; then
    echo "FAIL  CRLF (shebang będzie 'python\\r'): $f"; fail=1
  fi
done

echo "== Składnia Python 2"
for f in "${SCRIPTS[@]}"; do
  python2 -c "import sys; compile(open(sys.argv[1]).read(), sys.argv[1], 'exec')" "$f" \
    && echo "OK    $f" || { echo "FAIL  $f"; fail=1; }
done

echo "== Pliki launch"
for f in "${LAUNCHES[@]}"; do
  python2 -c "import sys, xml.dom.minidom as m; m.parse(sys.argv[1])" "$f" \
    && echo "OK    $f" || { echo "FAIL  $f"; fail=1; }
done

if [ "$fail" -ne 0 ]; then
  echo "== BŁĘDY w plikach — pomijam testy w ROS"
  exit 1
fi

# ------------------------------------------------------------------
echo "== roscore na porcie $PORT"
if rosnode list > /dev/null 2>&1; then
  echo "BŁĄD  na $ROS_MASTER_URI działa już ROS — test go nie użyje. Ustaw inny ROS_TEST_PORT."
  exit 2
fi
roscore -p "$PORT" > "$LOGS/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 50); do
  rosnode list > /dev/null 2>&1 && break
  kill -0 "$ROSCORE_PID" 2> /dev/null || break
  sleep 0.2
done
if ! kill -0 "$ROSCORE_PID" 2> /dev/null || ! rosnode list > /dev/null 2>&1; then
  echo "BŁĄD  roscore nie wystartował:"; tail -n 5 "$LOGS/roscore.log" | sed 's/^/      /'
  exit 2
fi

roslaunch --nodes lidar_threshold lidar_threshold.launch > /dev/null || { echo "FAIL  roslaunch --nodes lidar_threshold.launch"; exit 1; }

# ------------------------------------------------------------------
echo "== Start węzłów przez rosrun (sprawdza też shebang)"
rosrun cybus_robot emergency_stop_node.py > "$LOGS/cybus_emergency.log" 2>&1 & NODE_PIDS+=($!)
rosrun cybus_robot cybus_motion_node.py   > "$LOGS/cybus_motion.log"    2>&1 & NODE_PIDS+=($!)
rosrun lidar_threshold lidar_reader.py    > "$LOGS/lidar_reader.log"    2>&1 & NODE_PIDS+=($!)

missing=()
for _ in $(seq 75); do  # do 15 s
  list="$(rosnode list 2> /dev/null)"
  missing=(); dead=0
  for i in "${!NODE_NAMES[@]}"; do
    if ! grep -qx "/${NODE_NAMES[$i]}" <<< "$list"; then
      missing+=("${NODE_NAMES[$i]}")
      kill -0 "${NODE_PIDS[$i]}" 2> /dev/null || dead=1
    fi
  done
  [ "${#missing[@]}" -eq 0 ] || [ "$dead" -eq 1 ] && break
  sleep 0.2
done
if [ "${#missing[@]}" -ne 0 ]; then
  for n in "${missing[@]}"; do
    echo "FAIL  węzeł /$n nie wystartował — koniec $LOGS/$n.log:"
    tail -n 5 "$LOGS/$n.log" | sed 's/^/      /'
  done
  exit 1
fi
echo "OK    ${NODE_NAMES[*]}"

# ------------------------------------------------------------------
echo "== Scenariusze"
python2 -u "$HERE/test_estop_motion.py" > "$LOGS/test.log" 2>&1 &
TEST_PID=$!
# Wyniki na bieżąco; wait (a nie potok na pierwszym planie), żeby sygnał przerywał od razu
tail -n +1 -f --pid="$TEST_PID" "$LOGS/test.log" | grep --line-buffered -v '^\[' &
TAIL_PID=$!
wait "$TEST_PID" || fail=1
TEST_PID=""
wait "$TAIL_PID"

for n in "${NODE_NAMES[@]}"; do
  if grep -q -E "Traceback|UnicodeDecodeError" "$LOGS/$n.log"; then
    echo "FAIL  wyjątek w logu węzła /$n:"
    grep -A 8 -E "Traceback|UnicodeDecodeError" "$LOGS/$n.log" | head -n 12 | sed 's/^/      /'
    fail=1
  fi
done

[ "$fail" -eq 0 ] && echo "== OK" || echo "== BŁĘDY (logi: $LOGS)"
exit "$fail"
