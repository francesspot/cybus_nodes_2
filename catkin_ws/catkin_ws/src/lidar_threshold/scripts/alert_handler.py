#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String

pub_command = None

def alert_callback(msg):
    if msg.data:
        command = "STOP"
        rospy.logwarn("Obstacle detected! Sending: {}".format(command))
    else:
        command = "GO"

    pub_command.publish(String(command))

def main():
    global pub_command
    rospy.init_node('alert_handler', anonymous=False)
    pub_command = rospy.Publisher('/robot/command', String, queue_size=10)
    rospy.Subscriber('/lidar/alert', Bool, alert_callback)
    rospy.loginfo("alert_handler node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
