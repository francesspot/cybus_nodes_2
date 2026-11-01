#!/usr/bin/env python
import math

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool

# Alert turns on below THRESHOLD and off only above RELEASE_THRESHOLD (hysteresis),
# so a distance hovering around 0.5 m does not toggle the e-stop on every scan.
THRESHOLD         = 0.5  # meters
RELEASE_THRESHOLD = 0.6  # meters

# Only the front sector is checked. Angle 0 is the laser frame's x axis;
# set ~sector_center_deg if the lidar is mounted rotated relative to the robot.
SECTOR_CENTER_DEG = 0.0
SECTOR_WIDTH_DEG  = 120.0

pub_distance = None
pub_alert    = None
alert_active = False

def in_sector(angle, center, half_width):
    d = math.atan2(math.sin(angle - center), math.cos(angle - center))
    return abs(d) <= half_width

def scan_callback(msg):
    global alert_active

    center     = math.radians(SECTOR_CENTER_DEG)
    half_width = math.radians(SECTOR_WIDTH_DEG) / 2.0

    # REP 117: +inf = nothing within range_max, -inf = closer than range_min,
    # NaN = invalid. Zeros are treated as invalid, as most drivers use them for "no return".
    valid  = []
    no_hit = False
    for i, r in enumerate(msg.ranges):
        if not in_sector(msg.angle_min + i * msg.angle_increment, center, half_width):
            continue
        if r == float('inf'):
            no_hit = True
        elif r == float('-inf'):
            valid.append(0.0)
        elif 0.01 < r <= msg.range_max:
            valid.append(r)

    if valid:
        min_dist = min(valid)
    elif no_hit:
        min_dist = float('inf')
    else:
        # Nothing usable in the sector: publish nothing, so the e-stop watchdog
        # treats the lidar as unavailable instead of reporting a free path.
        rospy.logwarn_throttle(1.0, "No valid lidar ranges in the front sector.")
        return

    pub_distance.publish(Float32(min_dist))

    if alert_active:
        alert_active = min_dist < RELEASE_THRESHOLD
    else:
        alert_active = min_dist < THRESHOLD
    pub_alert.publish(Bool(alert_active))

    if alert_active:
        rospy.loginfo_throttle(1.0, "ALERT: Obstacle at {:.2f}m (threshold: {}m)".format(min_dist, THRESHOLD))

def main():
    global pub_distance, pub_alert, THRESHOLD, RELEASE_THRESHOLD, SECTOR_CENTER_DEG, SECTOR_WIDTH_DEG
    rospy.init_node('lidar_reader', anonymous=False)
    THRESHOLD         = rospy.get_param('~threshold',         THRESHOLD)
    RELEASE_THRESHOLD = rospy.get_param('~release_threshold', RELEASE_THRESHOLD)
    SECTOR_CENTER_DEG = rospy.get_param('~sector_center_deg', SECTOR_CENTER_DEG)
    SECTOR_WIDTH_DEG  = rospy.get_param('~sector_width_deg',  SECTOR_WIDTH_DEG)
    pub_distance = rospy.Publisher('/lidar/min_distance', Float32, queue_size=10)
    pub_alert    = rospy.Publisher('/lidar/alert',        Bool,    queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.loginfo("lidar_reader node started (front sector {:.0f} deg, alert < {}m, release > {}m).".format(
        SECTOR_WIDTH_DEG, THRESHOLD, RELEASE_THRESHOLD))
    rospy.spin()

if __name__ == '__main__':
    main()
