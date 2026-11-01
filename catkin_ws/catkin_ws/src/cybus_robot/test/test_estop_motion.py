#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test integracyjny e-stopu i ruchu (bez robota) — sprawdza założenia z ZALOZENIA.md.

Wymaga działających: roscore, cybus_motion, cybus_emergency, lidar_reader
(uruchamia je run_tests.sh). Test sam symuluje odometrię (całkuje /cmd_vel,
publikuje /odom_raw) i lidar (/lidar/alert co 100 ms, a w S11 /scan).

Kod wyjścia: 0 = wszystkie przypadki PASS, 1 = co najmniej jeden FAIL.
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

import rospy
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

lock = threading.Lock()
events = []
sim = {'lidar': False, 'alert': False, 'odom': 'move',
       'x': 0.0, 'y': 0.0, 's': 0.0, 'yaw': 0.0, 'yaw_total': 0.0, 'v': 0.0, 'w': 0.0}
results = []

def check(name, cond, info=''):
    results.append((name, cond))
    print('%s  %s  %s' % ('PASS' if cond else 'FAIL', name, info))

# ==============================================================================
# SUBSKRYPCJE I SYMULATORY
# ==============================================================================

def cv(m):
    with lock:
        events.append((time.time(), 'vel', m.linear.x, m.angular.z))
        sim['v'], sim['w'] = m.linear.x, m.angular.z
def es(m):
    with lock: events.append((time.time(), 'estop', m.data))
def la(m):
    with lock: events.append((time.time(), 'alert', m.data))
def md(m):
    with lock: events.append((time.time(), 'dist', m.data))

rospy.init_node('tester')

finished = [False]

def finish(code):
    finished[0] = True
    sys.exit(code)

def on_shutdown():
    # TERM/INT z run_tests.sh: zakończ od razu, zamiast dokańczać scenariusze bez węzłów
    if not finished[0]:
        os._exit(130)
rospy.on_shutdown(on_shutdown)
rospy.Subscriber('/cmd_vel', Twist, cv)
rospy.Subscriber('/emergency_stop', Bool, es)
cmd   = rospy.Publisher('/motion_cmd', String, queue_size=10)
alert = rospy.Publisher('/lidar/alert', Bool, queue_size=10)
odom  = rospy.Publisher('/odom_raw', Odometry, queue_size=10)
scan  = rospy.Publisher('/scan', LaserScan, queue_size=10)

def odom_loop():
    """'move' = całkuje /cmd_vel, 'stuck' = publikuje stałą pozę, 'off' = milczy."""
    dt = 0.02
    while not rospy.is_shutdown():
        with lock:
            mode = sim['odom']
            if mode == 'move':
                sim['x'] += sim['v'] * math.cos(sim['yaw']) * dt
                sim['y'] += sim['v'] * math.sin(sim['yaw']) * dt
                sim['s'] += sim['v'] * dt  # droga ze znakiem
                sim['yaw'] += sim['w'] * dt
                sim['yaw_total'] += sim['w'] * dt
            x, y, yaw = sim['x'], sim['y'], sim['yaw']
        if mode != 'off':
            o = Odometry()
            o.pose.pose.position.x = x
            o.pose.pose.position.y = y
            o.pose.pose.orientation.z = math.sin(yaw / 2)
            o.pose.pose.orientation.w = math.cos(yaw / 2)
            odom.publish(o)
        time.sleep(dt)

def lidar_loop():
    while not rospy.is_shutdown():
        if sim['lidar']:
            alert.publish(Bool(sim['alert']))
        time.sleep(0.1)

for f in (odom_loop, lidar_loop):
    t = threading.Thread(target=f)
    t.daemon = True
    t.start()

# Motion node subskrybuje /motion_cmd dopiero po pierwszej odometrii
deadline = time.time() + 30
while cmd.get_num_connections() < 1 or alert.get_num_connections() < 1:
    if rospy.is_shutdown() or time.time() > deadline:
        print('FAIL  węzły nie podłączyły się do /motion_cmd i /lidar/alert w 30 s')
        finish(1)
    time.sleep(0.1)
time.sleep(2.0)

# ==============================================================================
# HELPERS
# ==============================================================================

def send(action, value):
    cmd.publish(String(json.dumps({'action': action, 'value': value})))
def send_raw(s):
    cmd.publish(String(s))
def window(t0, t1=None):
    with lock: return [e for e in events if t0 <= e[0] < (t1 or time.time())]
def nonzero(evs):
    return [e for e in evs if e[1] == 'vel' and (e[2] != 0 or e[3] != 0)]
def estops(evs):
    return [e[2] for e in evs if e[1] == 'estop']
def last_estop():
    with lock:
        s = [e[2] for e in events if e[1] == 'estop']
    return s[-1] if s else None
def settle():
    send('ZATRZYMAJ_SIE', 0)
    time.sleep(0.5)

# ==============================================================================
# SCENARIUSZE
# ==============================================================================

print('=== S0 start: fail-safe przed danymi z lidaru')
check('S0 /emergency_stop=True przed lidarem', last_estop() is True, str(last_estop()))
t0 = time.time(); send('JEDZ_DO_PRZODU', 1.0); time.sleep(1.0)
check('S0 komenda odrzucona', len(nonzero(window(t0))) == 0)
t0 = time.time(); sim['lidar'] = True; time.sleep(0.5)
check('S0 alert=False -> odblokowanie', estops(window(t0)) == [False], str(estops(window(t0))))

print('=== S1 jazda 0.5 m kończy się na dystansie')
s0 = sim['s']; send('JEDZ_DO_PRZODU', 0.5); time.sleep(3.0)
check('S1 dystans', abs(sim['s'] - s0 - 0.5) < 0.05, 'ds=%.3f' % (sim['s'] - s0))

print('=== S2 obrót o 270 stopni w lewo (bez zawijania kąta)')
y0 = sim['yaw_total']; send('SKREC_W_LEWO', 270); time.sleep(8.0)
check('S2 kąt', abs(math.degrees(sim['yaw_total'] - y0) - 270) < 3,
      '%.1f st.' % math.degrees(sim['yaw_total'] - y0))

print('=== S3 e-stop przerywa jazdę; cofnięcie dozwolone')
send('JEDZ_DO_PRZODU', 2.0); time.sleep(1.0)
ta = time.time(); sim['alert'] = True; time.sleep(1.0)
w = window(ta)
te = [e[0] for e in w if e[1] == 'estop' and e[2]]
check('S3 jeden True na /emergency_stop', estops(w) == [True], str(estops(w)))
nz_after = [e for e in nonzero(w) if te and e[0] > te[0]]
check('S3 zero niezerowych prędkości po True', len(nz_after) == 0, 'po=%d' % len(nz_after))
nvel = len([e for e in w if e[1] == 'vel'])
check('S3 brak powtarzanych zer w pętli', nvel <= 4, 'wiadomości /cmd_vel=%d' % nvel)
t0 = time.time(); send('JEDZ_DO_PRZODU', 1.0); time.sleep(0.5); send('SKREC_W_LEWO', 90); time.sleep(0.7)
check('S3 jazda do przodu i obrót odrzucone', len(nonzero(window(t0))) == 0)
s0 = sim['s']; t0 = time.time(); send('JEDZ_DO_TYLU', 1.0); time.sleep(2.5)
nz = nonzero(window(t0))
check('S3 cofnięcie wykonane (do 0.3 m)', abs(sim['s'] - s0 + 0.3) < 0.04 and all(e[2] < 0 for e in nz),
      'ds=%.3f' % (sim['s'] - s0))
t0 = time.time(); sim['alert'] = False; time.sleep(1.0)
check('S3 po False brak wznowienia', len(nonzero(window(t0))) == 0 and estops(window(t0)) == [False])
t0 = time.time(); send('JEDZ_DO_PRZODU', 0.3); time.sleep(1.5)
check('S3 nowa komenda przyjęta', len(nonzero(window(t0))) > 0)
time.sleep(0.5)

print('=== S4 ZATRZYMAJ_SIE przerywa ruch i czyści kolejkę')
send('JEDZ_DO_PRZODU', 2.0); time.sleep(0.05); send('JEDZ_DO_PRZODU', 1.0); time.sleep(0.7)
tz = time.time(); send('ZATRZYMAJ_SIE', 0); time.sleep(2.0)
nz = nonzero(window(tz + 0.1))
check('S4 brak ruchu po stopie (także z kolejki)', len(nz) == 0, 'po=%d' % len(nz))

print('=== S5 krótki impuls alertu (True->False w 20 ms)')
send('JEDZ_DO_PRZODU', 2.0); time.sleep(0.7)
sim['lidar'] = False; time.sleep(0.15)
tp = time.time(); alert.publish(Bool(True)); time.sleep(0.02); alert.publish(Bool(False))
sim['lidar'] = True; time.sleep(1.0)
check('S5 jazda przerwana mimo impulsu', len(nonzero(window(tp + 0.15))) == 0,
      'estop=%s' % estops(window(tp)))
settle()

print('=== S6 watchdog: lidar milczy')
t0 = time.time(); sim['lidar'] = False; time.sleep(1.6)
te = [e[0] for e in window(t0) if e[1] == 'estop' and e[2]]
check('S6 e-stop po ciszy lidaru', len(te) == 1, 'po %.2f s' % (te[0] - t0) if te else '')
t0 = time.time(); send('JEDZ_DO_PRZODU', 0.3); time.sleep(0.7)
check('S6 komenda odrzucona', len(nonzero(window(t0))) == 0)
sim['lidar'] = True; time.sleep(0.5)
check('S6 powrót lidaru odblokowuje', last_estop() is False)

print('=== S7 odometria milczy w trakcie jazdy')
send('JEDZ_DO_PRZODU', 2.0); time.sleep(0.7)
to = time.time(); sim['odom'] = 'off'; time.sleep(1.5)
nz = nonzero(window(to))
check('S7 jazda przerwana', len(nz) > 0 and nz[-1][0] - to < 0.75,
      'ostatnia po %.2f s' % (nz[-1][0] - to if nz else -1))
sim['odom'] = 'move'; time.sleep(0.3)

print('=== S8 odometria stoi (np. poślizg): limit czasu')
sim['odom'] = 'stuck'; t0 = time.time(); send('JEDZ_DO_PRZODU', 0.1); time.sleep(4.0)
nz = nonzero(window(t0))
check('S8 przerwane po limicie', len(nz) > 0 and 2.3 < nz[-1][0] - t0 < 3.2,
      'ostatnia po %.2f s' % (nz[-1][0] - t0 if nz else -1))
sim['odom'] = 'move'; time.sleep(0.3)

print('=== S9 nieprawidłowe komendy')
for s in ['[1]', '{"action": 5, "value": 1}', '{"action": "JEDZ_DO_PRZODU", "value": "abc"}',
          '{"action": "JEDZ_DO_PRZODU", "value": NaN}', '{"action": "JEDZ_DO_PRZODU", "value": [1]}',
          'nie json', '{"action": "SKRĘĆ", "value": 1}']:
    send_raw(s); time.sleep(0.2)
t0 = time.time(); send('JEDZ_DO_PRZODU', 0.2); time.sleep(1.2)
check('S9 węzeł działa po złych komendach', len(nonzero(window(t0))) > 0)
settle()

print('=== S10 węzeł e-stopu pada')
send('JEDZ_DO_PRZODU', 2.0); time.sleep(0.7)
devnull = open('/dev/null', 'w')
tk = time.time(); subprocess.call(['rosnode', 'kill', '/cybus_emergency'], stdout=devnull); tk2 = time.time()
time.sleep(1.0)
nz = nonzero(window(tk2 + 0.2))
check('S10 jazda przerwana po awarii e-stopu', len(nz) == 0, 'kill trwał %.2f s, po=%d' % (tk2 - tk, len(nz)))
t0 = time.time(); send('JEDZ_DO_PRZODU', 0.3); time.sleep(0.7)
check('S10 komenda odrzucona bez węzła e-stopu', len(nonzero(window(t0))) == 0)

print('=== S11 lidar_reader: sektor, histereza, inf/NaN')
sim['lidar'] = False
rospy.Subscriber('/lidar/alert', Bool, la)
rospy.Subscriber('/lidar/min_distance', Float32, md)
deadline = time.time() + 10
while scan.get_num_connections() < 1:
    if rospy.is_shutdown() or time.time() > deadline:
        print('FAIL  lidar_reader nie podłączył się do /scan w 10 s')
        finish(1)
    time.sleep(0.1)
time.sleep(1.0)

def mkscan(default, rays):
    """Skan 360 promieni co 1 st.; rays = [(kąt w st., odległość)], każdy na 7 promieni."""
    s = LaserScan()
    s.angle_min = -math.pi
    s.angle_increment = 2 * math.pi / 360
    s.angle_max = math.pi - s.angle_increment
    s.range_min = 0.15
    s.range_max = 12.0
    r = [default] * 360
    for deg, val in rays:
        for d in range(deg - 3, deg + 4):
            r[(d + 180) % 360] = val
    s.ranges = r
    return s

def feed(default, rays):
    t0 = time.time(); scan.publish(mkscan(default, rays)); time.sleep(0.3)
    a = [e[2] for e in window(t0) if e[1] == 'alert']
    d = [e[2] for e in window(t0) if e[1] == 'dist']
    return (a[-1] if a else None), (d[-1] if d else None)

inf, nan = float('inf'), float('nan')
cases = [('tył 0.3 m -> brak alertu', inf, [(180, 0.3)], False),
         ('przód 0.4 -> alert', inf, [(0, 0.4)], True),
         ('przód 0.55 -> dalej alert (histereza)', inf, [(0, 0.55)], True),
         ('przód 0.65 -> koniec alertu', inf, [(0, 0.65)], False),
         ('przód 0.55 -> brak alertu', inf, [(0, 0.55)], False),
         ('bok 70 st. 0.3 -> poza sektorem', inf, [(70, 0.3)], False),
         ('bok 50 st. 0.3 -> w sektorze', inf, [(50, 0.3)], True),
         ('same inf -> wolne', inf, [], False),
         ('-inf z przodu -> alert', inf, [(0, -inf)], True),
         ('same NaN -> brak publikacji', nan, [], None)]
for name, default, rays, exp in cases:
    a, d = feed(default, rays)
    check('S11 ' + name, a == exp, 'alert=%s dist=%s' % (a, d))

passed = sum(1 for _, c in results if c)
print('=== WYNIK: %d/%d PASS' % (passed, len(results)))
finish(0 if passed == len(results) else 1)
