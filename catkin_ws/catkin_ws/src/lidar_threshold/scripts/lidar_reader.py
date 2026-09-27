#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool

THRESHOLD = 0.5  # meters

pub_distance = None
pub_alert    = None

def scan_callback(msg):
    valid = [r for r in msg.ranges if 0.01 < r < msg.range_max]

    if not valid:
        rospy.logwarn("No valid lidar ranges received.")
        return

    min_dist = min(valid)
    pub_distance.publish(Float32(min_dist))

    triggered = min_dist < THRESHOLD
    pub_alert.publish(Bool(triggered))

    if triggered:
        rospy.loginfo("ALERT: Obstacle at {:.2f}m (threshold: {}m)".format(min_dist, THRESHOLD))

def main():
    global pub_distance, pub_alert
    rospy.init_node('lidar_reader', anonymous=False)
    pub_distance = rospy.Publisher('/lidar/min_distance', Float32, queue_size=10)
    pub_alert    = rospy.Publisher('/lidar/alert',        Bool,    queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.loginfo("lidar_reader node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
