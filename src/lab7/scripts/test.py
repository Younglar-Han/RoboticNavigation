#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

def callback(msg):
    max_intensity = 0
    for i in range(len(msg.ranges)):
        if msg.ranges[i] != 0.0:
            if msg.intensities[i]*msg.ranges[i] > max_intensity:
                max_intensity = msg.intensities[i]*msg.ranges[i]
                max_index = i
    print(max_intensity)
    print(max_index)


def pole_follower():

    rospy.init_node('pole_follower', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    pole_follower()
