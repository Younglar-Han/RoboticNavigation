 #! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
scann = LaserScan()

def callback(msg):
    #print(len(msg.ranges)) len is 2019 from 0-360
    current_time = rospy.Time.now()
    scann.header.stamp = current_time
    scann.header.frame_id = 'laser'
    scann.angle_min = 0.0
    scann.angle_max = 6.28318458203
    scann.angle_increment = 0.0174532923847
    scann.time_increment = 2.98899994959e-05
    scann.scan_time = 0.0
    scann.range_min = 0.119999997318
    scann.range_max = 3.5
    scann.ranges = msg.ranges[0:72]
    scann.intensities = msg.intensities[0:72]
    print(scann)
    pub.publish(scann)

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()