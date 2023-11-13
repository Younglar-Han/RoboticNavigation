#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


def callback(msg):
    range_min = 10.0
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < range_min and msg.ranges[i] != 0.0:
            range_min = msg.ranges[i]
            range_min_index = i
    print('min:', range_min, 'index: ', range_min_index)
    # rospy.loginfo(rospy.get_caller_id() + 'I heard')


def pole_follower():

    rospy.init_node('pole_follower', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    pole_follower()
