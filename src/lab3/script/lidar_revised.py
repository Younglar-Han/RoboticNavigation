 #! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
scann = LaserScan()
real_range_max_golbal = 0.0
real_range_min_global = 10.0

def callback(msg):
    global real_range_max_golbal
    global real_range_min_global
    current_time = rospy.Time.now()
    scann.header.stamp = current_time
    scann.header.frame_id = 'laser'
    scann.angle_min = 0.0
    scann.angle_max = 6.28318458203
    scann.angle_increment = 0.0174532923847
    scann.time_increment = 2.98899994959e-05
    scann.scan_time = msg.scan_time
    scann.range_min = 0.119999997318
    scann.range_max = 3.5
    scann.ranges = msg.ranges
    scann.intensities = msg.intensities
    real_range_max = max(msg.ranges)
    
    real_range_min = 10.0
    for i in range(0, len(msg.ranges)):
        if msg.ranges[i] < real_range_min and msg.ranges[i] > 0.01:
            real_range_min = msg.ranges[i]
    if real_range_max > real_range_max_golbal:
        real_range_max_golbal = real_range_max
    if real_range_min < real_range_min_global:
        real_range_min_global = real_range_min
    print('real range max =', real_range_max_golbal, '  real range min =', real_range_min_global)
    
    # pub.publish(scann)

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()