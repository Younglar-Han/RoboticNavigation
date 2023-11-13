#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    range_min = 10.0
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < range_min and msg.ranges[i] != 0.0:
            range_min = msg.ranges[i]
            range_min_index = i
    print('min:', range_min, 'index: ', range_min_index)

    twist = Twist()
    if (range_min < 1.5 and range_min > 0.2):
        if range_min_index > 10 and range_min_index < 180:
            twist.angular.z = (range_min_index - 10)/180.0 * 1.7 + 0.5
        elif range_min_index > 180 and range_min_index < 350:
            twist.angular.z = (range_min_index - 350)/180.0 * 1.7 - 0.5
        elif range_min_index > 350 or range_min_index < 10:
            twist.linear.x = 0.21
    print(twist.angular.z)
    pub.publish(twist)


def pole_follower():

    rospy.init_node('pole_follower', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    pole_follower()
