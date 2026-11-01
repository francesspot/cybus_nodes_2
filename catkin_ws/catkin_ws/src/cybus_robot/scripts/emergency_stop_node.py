#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
emergency_stop_node — zatrzymanie awaryjne na podstawie lidaru.

Subskrybuje: /lidar/alert      (std_msgs/Bool) — True = przeszkoda w przednim sektorze
Publikuje:   /emergency_stop   (std_msgs/Bool, latched) — aktualny stan e-stopu
             /cmd_vel          (geometry_msgs/Twist)    — zero przy włączeniu e-stopu

Węzeł nie nasłuchuje komend tekstowych: jedynym wyzwalaczem jest /lidar/alert.
Stan publikowany jest tylko przy zmianie (lidar wysyła stan przy każdym skanie).

Fail-safe: e-stop jest aktywny od startu, dopóki lidar nie potwierdzi wolnej
drogi, i włącza się, gdy /lidar/alert milczy dłużej niż ~alert_timeout.

Zero na /cmd_vel idzie raz, przy włączeniu e-stopu, a nie w pętli — ciągłe zera
blokowałyby wycofanie się robota od przeszkody (motion node przyjmuje podczas
e-stopu krótkie JEDZ_DO_TYLU). Ruch blokuje sam motion node.

Parametry ROS:
  ~alert_timeout (float, 1.0) — po ilu sekundach ciszy na /lidar/alert włączyć e-stop
"""

import threading

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class EmergencyStopNode(object):

    def __init__(self):
        rospy.init_node('cybus_emergency', anonymous=False)

        self._alert_timeout = rospy.get_param("~alert_timeout", 1.0)

        # Lock chroni stan i kolejność publikacji (callback lidaru i watchdog
        # działają w różnych wątkach)
        self._lock = threading.Lock()
        self._stopped = None
        self._last_alert = None

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Latched: węzeł uruchomiony później od razu dostaje bieżący stan
        self._estop_pub = rospy.Publisher('/emergency_stop', Bool,
                                          queue_size=1, latch=True)
        with self._lock:
            self._set_state(True, "czekam na pierwsze dane z /lidar/alert")

        rospy.Subscriber('/lidar/alert', Bool, self._alert_callback, queue_size=1)

        rospy.loginfo("[E-stop] Uruchomiony. Nasłuchuję /lidar/alert...")

    def _alert_callback(self, msg):
        with self._lock:
            self._last_alert = rospy.get_time()
            if msg.data:
                self._set_state(True, "przeszkoda na drodze")
            else:
                self._set_state(False, "droga wolna")

    def _set_state(self, stopped, reason):
        """Wymaga trzymania self._lock."""
        if stopped == self._stopped:
            return  # lidar publikuje stan przy każdym skanie; reaguje się tylko na zmianę
        self._stopped = stopped

        if stopped:
            rospy.logwarn("[E-stop] ZATRZYMANIE AWARYJNE — %s!", reason)
            self._stop_robot()
        else:
            rospy.loginfo("[E-stop] Ruch odblokowany — %s.", reason)

        self._estop_pub.publish(Bool(stopped))

    def _stop_robot(self):
        self._cmd_vel_pub.publish(Twist())

    def spin(self):
        # Watchdog: brak świeżych danych z lidaru = brak ochrony, więc e-stop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self._lock:
                if (self._last_alert is not None and
                        rospy.get_time() - self._last_alert > self._alert_timeout):
                    self._set_state(True, "brak danych z lidaru (> %.1f s)" % self._alert_timeout)
            rate.sleep()


if __name__ == '__main__':
    try:
        EmergencyStopNode().spin()
    except rospy.ROSInterruptException:
        pass
