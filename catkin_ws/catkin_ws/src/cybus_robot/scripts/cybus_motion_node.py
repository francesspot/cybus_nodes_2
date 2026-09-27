#!/usr/bin/env python3
"""
Subskrybuje: /motion_cmd  (std_msgs/String)  — JSON z akcją i wartością
Publikuje:   /cmd_vel     (geometry_msgs/Twist)

Format komendy (JSON string):
  {"action": "JEDZ_DO_PRZODU", "value": 1.0}   <- metry
  {"action": "SKREC_W_PRAWO",  "value": 90.0}  <- stopnie
  {"action": "ZATRZYMAJ_SIE",  "value": 0}

Parametry ROS:
  ~linear_speed    (float, 0.3)  — prędkość liniowa [m/s]
  ~angular_speed   (float, 0.8)  — prędkość kątowa [rad/s]
  ~dist_tolerance  (float, 0.02) — tolerancja dystansu [m]
  ~angle_tolerance (float, 1.0)  — tolerancja kąta [stopnie]
"""

import json
import math
import threading

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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

# ==============================================================================
# WĘZEŁ
# ==============================================================================

class CybusMotionNode:

    def __init__(self):
        rospy.init_node('cybus_motion', anonymous=False)

        # Parametry
        self._linear_speed    = rospy.get_param("~linear_speed",    0.3)
        self._angular_speed   = rospy.get_param("~angular_speed",   0.8)
        self._dist_tolerance  = rospy.get_param("~dist_tolerance",  0.02)
        self._angle_tolerance = rospy.get_param("~angle_tolerance", 1.0)   # stopnie

        # Stan odometrii (aktualizowany z /odom_combined)
        self._odom_lock = threading.Lock()
        self._x   = None
        self._y   = None
        self._yaw = None

        # Publisher prędkości
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subskrypcja odometrii i komend
        rospy.Subscriber('/odom_raw',  Odometry,
                         self._odom_callback, queue_size=10)
        rospy.Subscriber('/motion_cmd', String,
                         self._cmd_callback,  queue_size=20)

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("[Motion] Czekam na pierwszą wiadomość z /odom_combined...")
        while not rospy.is_shutdown():
            with self._odom_lock:
                if self._x is not None:
                    break
            rospy.sleep(0.1)

        rospy.loginfo("[Motion] Gotowy. linear=%.2f m/s  angular=%.2f rad/s",
                      self._linear_speed, self._angular_speed)

    # ------------------------------------------------------------------
    def _odom_callback(self, msg):
        with self._odom_lock:
            p = msg.pose.pose.position
            self._x   = p.x
            self._y   = p.y
            self._yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def _get_pose(self):
        with self._odom_lock:
            return self._x, self._y, self._yaw

    # ------------------------------------------------------------------
    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_vel(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._cmd_pub.publish(t)

    # ------------------------------------------------------------------
    def _drive_distance(self, distance_m):
        """Jedź do przodu/tyłu o zadany dystans [m] z kontrolą odometryczną."""
        sign   = 1.0 if distance_m >= 0 else -1.0
        target = abs(distance_m)

        x0, y0, _ = self._get_pose()
        rate = rospy.Rate(20)

        rospy.loginfo("[Motion] Jazda %.2f m", distance_m)

        while not rospy.is_shutdown():
            x, y, _ = self._get_pose()
            travelled = math.sqrt((x - x0)**2 + (y - y0)**2)
            remaining = target - travelled

            if remaining <= self._dist_tolerance:
                break

            # Zwolnienie przy dojeżdżaniu
            speed = self._linear_speed
            if remaining < 0.1:
                speed *= 0.5

            self._publish_vel(linear=sign * speed)
            rate.sleep()

        self._stop()
        rospy.loginfo("[Motion] Dystans osiągnięty.")

    def _rotate_degrees(self, degrees):
        """Obróć o zadany kąt [stopnie], dodatni = lewo, ujemny = prawo."""
        target_rad = math.radians(degrees)
        tol_rad    = math.radians(self._angle_tolerance)

        _, _, yaw0 = self._get_pose()
        rotated    = 0.0
        prev_yaw   = yaw0
        rate       = rospy.Rate(20)

        rospy.loginfo("[Motion] Obrót %.1f stopni", degrees)

        while not rospy.is_shutdown():
            _, _, yaw = self._get_pose()
            delta    = angle_diff(yaw, prev_yaw)
            rotated += delta
            prev_yaw = yaw

            remaining = angle_diff(target_rad, rotated)

            if abs(remaining) <= tol_rad:
                break

            sign  = 1.0 if remaining > 0 else -1.0
            speed = self._angular_speed
            if abs(remaining) < math.radians(15):
                speed *= 0.5

            self._publish_vel(angular=sign * speed)
            rate.sleep()

        self._stop()
        rospy.loginfo("[Motion] Obrót zakończony.")

    # ------------------------------------------------------------------
    def _cmd_callback(self, msg):
        raw = msg.data.strip()
        rospy.loginfo("[Motion] Odebrano: %s", raw)

        try:
            data = json.loads(raw)
        except ValueError:
            rospy.logerr("[Motion] Nieprawidłowy JSON: %s", raw)
            return

        action = data.get("action", "")
        value  = float(data.get("value", 0.0))

        if action == "JEDZ_DO_PRZODU":
            self._drive_distance(value)
        elif action == "JEDZ_DO_TYLU":
            self._drive_distance(-value)
        elif action == "SKREC_W_LEWO":
            self._rotate_degrees(value)
        elif action == "SKREC_W_PRAWO":
            self._rotate_degrees(-value)
        elif action == "ZATRZYMAJ_SIE":
            self._stop()
            rospy.loginfo("[Motion] Stop.")
        else:
            rospy.logwarn("[Motion] Nieznana akcja: %s — zatrzymuję.", action)
            self._stop()

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
        passython3
# -*- coding: utf-8 -*-
"""
cybus_motion_node — węzeł ROS sterujący silnikami JetBota.

Subskrybuje: /motion_cmd  (std_msgs/String)
  Obsługiwane komendy:
    "jedz prosto"   → jazda do przodu
    "jedz do tylu"  → jazda do tyłu
    "skrec w lewo"  → skręt w lewo (w miejscu)
    "skrec w prawo" → skręt w prawo (w miejscu)
    "zatrzymaj sie" → stop

Parametry ROS:
    ~speed        (float, domyślnie 0.4)  — prędkość bazowa silników [0.0–1.0]
    ~turn_speed   (float, domyślnie 0.35) — prędkość podczas skrętu
"""

import rospy
from std_msgs.msg import String

try:
    from jetbot import Robot
except ImportError:
    rospy.logwarn(
        "[Motion] Biblioteka jetbot niedostępna — używam trybu symulacji (mock)."
    )
    # Prosty mock żeby węzeł dało się uruchomić i testować bez sprzętu
    class Robot:
        def forward(self, speed):  rospy.loginfo("[MOCK] forward(%.2f)", speed)
        def backward(self, speed): rospy.loginfo("[MOCK] backward(%.2f)", speed)
        def left(self, speed):     rospy.loginfo("[MOCK] left(%.2f)", speed)
        def right(self, speed):    rospy.loginfo("[MOCK] right(%.2f)", speed)
        def stop(self):            rospy.loginfo("[MOCK] stop()")

# ==============================================================================
# WĘZEŁ
# ==============================================================================

class CybusMotionNode:
    # Mapowanie komend tekstowych na metody robota
    _COMMAND_MAP = {
        "jedz prosto":   "forward",
        "jedz do tylu":  "backward",
        "skrec w lewo":  "left",
        "skrec w prawo": "right",
        "zatrzymaj sie": "stop",
    }

    def __init__(self):
        rospy.init_node('cybus_motion', anonymous=False)

        self._speed      = rospy.get_param("~speed",      0.4)
        self._turn_speed = rospy.get_param("~turn_speed", 0.35)

        rospy.loginfo("[Motion] Inicjalizacja robota...")
        self._robot = Robot()
        rospy.loginfo("[Motion] Robot gotowy. Prędkość: %.2f, skręt: %.2f",
                      self._speed, self._turn_speed)

        self._sub = rospy.Subscriber(
                '/motion_cmd', String, self._cmd_callback, queue_size=20
        )

        # Bezpieczne zatrzymanie przy zamknięciu węzła
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("[Motion] Subskrybuję /motion_cmd — gotowy na komendy.")

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        rospy.loginfo("[Motion] Odebrano komendę: '%s'", cmd)

        action = self._COMMAND_MAP.get(cmd)

        if action is None:
            rospy.logwarn("[Motion] Nieznana komenda: '%s' — zatrzymuję robota.", cmd)
            self._robot.stop()
            return

        if action == "stop":
            self._robot.stop()
        elif action in ("left", "right"):
            getattr(self._robot, action)(self._turn_speed)
        else:
            getattr(self._robot, action)(self._speed)

    def _on_shutdown(self):
        rospy.loginfo("[Motion] Zamykanie węzła — zatrzymuję robota.")
        try:
            self._robot.stop()
        except Exception:
            pass

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
