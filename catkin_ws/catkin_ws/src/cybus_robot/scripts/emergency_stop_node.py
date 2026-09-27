#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist

is_emergency_stopped = False
cmd_vel_pub = None

def command_callback(msg):
    global is_emergency_stopped
    raw_data = msg.data.strip()
    
    command = ""
    try:
        data = json.loads(raw_data)
        command = data.get("action", "").lower()
    except ValueError:
        command = raw_data.lower()
    
    if command in ["robot stop", "stop", "zatrzymaj_sie", "zatrzymaj sie"]:
        rospy.logwarn("OTRZYMANO SYGNAL AWARYJNY: ZATRZYMANIE ROBOTA!")
        is_emergency_stopped = True
        stop_robot()
    elif command in ["robot go", "start", "jedz prosto", "jedz_do_przodu"]:
        rospy.loginfo("Odblokowano ruch robota.")
        is_emergency_stopped = False

def stop_robot():
    stop_msg = Twist()
    cmd_vel_pub.publish(stop_msg)

def main():
    global cmd_vel_pub
    rospy.init_node('cybus_emergency_node', anonymous=True)
    
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber('/motion_cmd', String, command_callback)
    
    rospy.loginfo("Node cybus_emergency uruchomiony. Nasluchiwanie na /motion_cmd...")
    
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        if is_emergency_stopped:
            stop_robot()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass