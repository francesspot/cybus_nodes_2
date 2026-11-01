#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Subskrybuje: /motion_cmd      (std_msgs/String) — JSON z akcją i wartością
             /emergency_stop  (std_msgs/Bool)   — stan e-stopu z emergency_stop_node
Publikuje:   /cmd_vel         (geometry_msgs/Twist)

Format komendy (JSON string):
  {"action": "JEDZ_DO_PRZODU", "value": 1.0}   <- metry
  {"action": "SKREC_W_PRAWO",  "value": 90.0}  <- stopnie
  {"action": "ZATRZYMAJ_SIE",  "value": 0}

Wykonanie:
  Callback /motion_cmd tylko waliduje komendę i wstawia ją do kolejki; ruchy
  wykonuje po kolei jeden wątek roboczy. Dzięki temu ZATRZYMAJ_SIE przerywa
  trwający ruch od razu i czyści kolejkę, a komendy z kilku publisherów nie
  wykonują się równolegle.

Zatrzymanie awaryjne:
  rospy wywołuje callbacki w wątku danego połączenia, więc callback
  /emergency_stop nie czeka na koniec ruchu. Ustawia flagę, którą pętle ruchu
  sprawdzają w każdej iteracji razem z publikacją prędkości (pod jednym lockiem,
  więc po zerze z e-stopu nie wyjdzie już żadna niezerowa prędkość). Ruch zostaje
  przerwany najpóźniej po 1/20 s i nie jest wznawiany po odblokowaniu; komendy
  z kolejki przyjęte przed e-stopem są porzucane.
  Podczas e-stopu odrzucane są wszystkie komendy ruchu poza JEDZ_DO_TYLU,
  skróconym do ~max_escape_distance — lidar patrzy tylko do przodu, więc
  cofnięcie pozwala odjechać od przeszkody. Ruch jest blokowany także wtedy,
  gdy węzeł e-stopu nie działa (brak publishera /emergency_stop).

Parametry ROS:
  ~linear_speed        (float, 0.3)  — prędkość liniowa [m/s]
  ~angular_speed       (float, 0.8)  — prędkość kątowa [rad/s]
  ~dist_tolerance      (float, 0.02) — tolerancja dystansu [m]
  ~angle_tolerance     (float, 1.0)  — tolerancja kąta [stopnie]
  ~max_escape_distance (float, 0.3)  — maks. cofnięcie podczas e-stopu [m]
  ~odom_timeout        (float, 0.5)  — przerwij ruch, gdy odometria milczy dłużej [s]
  ~timeout_factor      (float, 2.0)  — limit czasu ruchu = factor * czas nominalny + 2 s

Kod działa w Pythonie 2 i 3; shebang `python` = 2.7 na Melodic.
"""

import json
import math
import threading

try:
    import queue
except ImportError:  # Python 2
    import Queue as queue

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    string_types = basestring  # Python 2: str i unicode
except NameError:
    string_types = str

# ==============================================================================
# HELPERS
# ==============================================================================

def quaternion_to_yaw(q):
    """Konwertuje kwaternion na kąt yaw w radianach."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    """Różnica kątów w radianach, wynik w zakresie (-pi, pi)."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class Job(object):
    """Komenda ruchu w kolejce wraz ze stanem, w którym ją przyjęto."""

    def __init__(self, action, kind, value, stop_gen, estop_gen, escape):
        self.action    = action
        self.kind      = kind       # "drive" [m] albo "rotate" [stopnie]
        self.value     = value
        self.stop_gen  = stop_gen
        self.estop_gen = estop_gen
        self.escape    = escape     # cofnięcie podczas e-stopu

# ==============================================================================
# WĘZEŁ
# ==============================================================================

class CybusMotionNode(object):

    def __init__(self):
        rospy.init_node('cybus_motion', anonymous=False)

        # Parametry
        self._linear_speed    = rospy.get_param("~linear_speed",    0.3)
        self._angular_speed   = rospy.get_param("~angular_speed",   0.8)
        self._dist_tolerance  = rospy.get_param("~dist_tolerance",  0.02)
        self._angle_tolerance = rospy.get_param("~angle_tolerance", 1.0)   # stopnie
        self._max_escape      = rospy.get_param("~max_escape_distance", 0.3)
        self._odom_timeout    = rospy.get_param("~odom_timeout",    0.5)
        self._timeout_factor  = rospy.get_param("~timeout_factor",  2.0)

        # Stan odometrii (aktualizowany z /odom_raw)
        self._odom_lock = threading.Lock()
        self._x     = None
        self._y     = None
        self._yaw   = None
        self._stamp = None

        # Lock chroni liczniki, bieżący ruch i publikację prędkości ruchu.
        # Event ustawiany w wątku /emergency_stop; fail-safe: aktywny do
        # pierwszego False z węzła e-stopu.
        self._lock      = threading.Lock()
        self._estop     = threading.Event()
        self._estop.set()
        self._stop_gen  = 0      # ++ przy ZATRZYMAJ_SIE: unieważnia ruch i kolejkę
        self._estop_gen = 0      # ++ przy włączeniu e-stopu
        self._current   = None
        self._jobs      = queue.Queue()

        # Publisher prędkości
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subskrypcje
        rospy.Subscriber('/odom_raw',  Odometry,
                         self._odom_callback, queue_size=10)
        self._estop_sub = rospy.Subscriber('/emergency_stop', Bool,
                                           self._estop_callback, queue_size=1)

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("[Motion] Czekam na pierwszą wiadomość z /odom_raw...")
        while not rospy.is_shutdown():
            with self._odom_lock:
                if self._x is not None:
                    break
            rospy.sleep(0.1)

        worker = threading.Thread(target=self._worker_loop)
        worker.daemon = True
        worker.start()

        # Komendy dopiero po odometrii — wcześniej nie byłoby punktu odniesienia
        rospy.Subscriber('/motion_cmd', String,
                         self._cmd_callback,  queue_size=20)

        rospy.loginfo("[Motion] Gotowy. linear=%.2f m/s  angular=%.2f rad/s",
                      self._linear_speed, self._angular_speed)

    # ------------------------------------------------------------------
    def _odom_callback(self, msg):
        with self._odom_lock:
            p = msg.pose.pose.position
            self._x     = p.x
            self._y     = p.y
            self._yaw   = quaternion_to_yaw(msg.pose.pose.orientation)
            self._stamp = rospy.get_time()

    def _get_pose(self):
        with self._odom_lock:
            return self._x, self._y, self._yaw, self._stamp

    def _estop_callback(self, msg):
        with self._lock:
            if msg.data:
                self._estop.set()
                self._estop_gen += 1
                # Cofnięcie od przeszkody nie jest przerywane
                if self._current is None or not self._current.escape:
                    self._stop()
            else:
                self._estop.clear()

    def _blocked(self):
        """E-stop aktywny albo węzeł e-stopu nie działa. Wymaga self._lock."""
        return self._estop.is_set() or self._estop_sub.get_num_connections() == 0

    def _abort_reason(self, job):
        """Powód przerwania ruchu albo None. Wymaga self._lock."""
        if job.stop_gen != self._stop_gen:
            return "ZATRZYMAJ_SIE"
        if job.escape:
            return None
        if self._blocked() or job.estop_gen != self._estop_gen:
            return "e-stop"
        return None

    # ------------------------------------------------------------------
    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_vel(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._cmd_pub.publish(t)

    def _publish_motion(self, job, linear=0.0, angular=0.0):
        """Sprawdza przerwanie i publikuje prędkość atomowo. Zwraca powód przerwania albo None."""
        with self._lock:
            reason = self._abort_reason(job)
            if reason is None:
                self._publish_vel(linear, angular)
            return reason

    def _check_progress(self, stamp, deadline):
        now = rospy.get_time()
        if now - stamp > self._odom_timeout:
            return "brak odometrii"
        if now > deadline:
            return "przekroczenie czasu"
        return None

    # ------------------------------------------------------------------
    def _drive_distance(self, job, distance_m):
        """Jedź do przodu/tyłu o zadany dystans [m] z kontrolą odometryczną."""
        sign   = 1.0 if distance_m >= 0 else -1.0
        target = abs(distance_m)

        x0, y0, _, _ = self._get_pose()
        deadline = (rospy.get_time() + 2.0 +
                    self._timeout_factor * target / self._linear_speed)
        rate = rospy.Rate(20)

        rospy.loginfo("[Motion] Jazda %.2f m%s", distance_m,
                      " (cofnięcie podczas e-stopu)" if job.escape else "")

        while not rospy.is_shutdown():
            x, y, _, stamp = self._get_pose()
            travelled = math.sqrt((x - x0)**2 + (y - y0)**2)
            remaining = target - travelled

            if remaining <= self._dist_tolerance:
                break

            # Zwolnienie przy dojeżdżaniu
            speed = self._linear_speed
            if remaining < 0.1:
                speed *= 0.5

            reason = (self._check_progress(stamp, deadline) or
                      self._publish_motion(job, linear=sign * speed))
            if reason:
                self._stop()
                rospy.logwarn("[Motion] Jazda przerwana przez %s.", reason)
                return
            rate.sleep()

        self._stop()
        rospy.loginfo("[Motion] Dystans osiągnięty.")

    def _rotate_degrees(self, job, degrees):
        """Obróć o zadany kąt [stopnie], dodatni = lewo, ujemny = prawo."""
        target_rad = math.radians(degrees)
        tol_rad    = math.radians(self._angle_tolerance)

        _, _, yaw0, _ = self._get_pose()
        rotated    = 0.0
        prev_yaw   = yaw0
        deadline   = (rospy.get_time() + 2.0 +
                      self._timeout_factor * abs(target_rad) / self._angular_speed)
        rate       = rospy.Rate(20)

        rospy.loginfo("[Motion] Obrót %.1f stopni", degrees)

        while not rospy.is_shutdown():
            _, _, yaw, stamp = self._get_pose()
            delta    = angle_diff(yaw, prev_yaw)
            rotated += delta
            prev_yaw = yaw

            # Bez zawijania: `rotated` jest sumowane, więc obrót > 180° działa
            remaining = target_rad - rotated

            if abs(remaining) <= tol_rad:
                break

            sign  = 1.0 if remaining > 0 else -1.0
            speed = self._angular_speed
            if abs(remaining) < math.radians(15):
                speed *= 0.5

            reason = (self._check_progress(stamp, deadline) or
                      self._publish_motion(job, angular=sign * speed))
            if reason:
                self._stop()
                rospy.logwarn("[Motion] Obrót przerwany przez %s.", reason)
                return
            rate.sleep()

        self._stop()
        rospy.loginfo("[Motion] Obrót zakończony.")

    # ------------------------------------------------------------------
    def _worker_loop(self):
        while not rospy.is_shutdown():
            try:
                job = self._jobs.get(timeout=0.1)
            except queue.Empty:
                continue

            with self._lock:
                reason = self._abort_reason(job)
                if reason is None:
                    self._current = job
            if reason:
                rospy.logwarn("[Motion] Pomijam komendę %s z kolejki (%s).",
                              job.action, reason)
                continue

            try:
                if job.kind == "drive":
                    self._drive_distance(job, job.value)
                else:
                    self._rotate_degrees(job, job.value)
            finally:
                with self._lock:
                    self._current = None

    def _cmd_callback(self, msg):
        raw = msg.data.strip()
        rospy.loginfo("[Motion] Odebrano: %s", raw)

        try:
            data   = json.loads(raw)
            action = data.get("action", "")
            value  = float(data.get("value", 0.0))
        except (ValueError, TypeError, AttributeError):
            rospy.logerr("[Motion] Nieprawidłowa komenda: %s", raw)
            return

        if (not isinstance(action, string_types) or
                math.isnan(value) or math.isinf(value)):
            rospy.logerr("[Motion] Nieprawidłowa komenda: %s", raw)
            return

        # Python 2: json zwraca unicode, a literały w pliku to bajty UTF-8;
        # sklejenie obu w logu z polskimi znakami rzuca UnicodeDecodeError
        if not isinstance(action, str):
            action = action.encode("utf-8")

        if action == "JEDZ_DO_PRZODU":
            kind, value = "drive", value
        elif action == "JEDZ_DO_TYLU":
            kind, value = "drive", -value
        elif action == "SKREC_W_LEWO":
            kind, value = "rotate", value
        elif action == "SKREC_W_PRAWO":
            kind, value = "rotate", -value
        else:
            # ZATRZYMAJ_SIE i nieznane akcje przerywają ruch i czyszczą kolejkę
            with self._lock:
                self._stop_gen += 1
                self._stop()
            if action == "ZATRZYMAJ_SIE":
                rospy.loginfo("[Motion] Stop.")
            else:
                rospy.logwarn("[Motion] Nieznana akcja: %s — zatrzymuję.", action)
            return

        with self._lock:
            blocked = self._blocked()
            escape  = blocked and kind == "drive" and value < 0
            if blocked and not escape:
                rospy.logwarn("[Motion] E-stop aktywny — odrzucam komendę %s.", action)
                return
            if escape and -value > self._max_escape:
                rospy.logwarn("[Motion] E-stop aktywny — cofnięcie ograniczone do %.2f m.",
                              self._max_escape)
                value = -self._max_escape
            self._jobs.put(Job(action, kind, value,
                               self._stop_gen, self._estop_gen, escape))

    def _on_shutdown(self):
        rospy.loginfo("[Motion] Zamykanie — stop.")
        self._stop()

    def spin(self):
        rospy.spin()


def main():
    node = CybusMotionNode()
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
