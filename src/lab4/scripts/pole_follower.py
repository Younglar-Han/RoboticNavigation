#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# import numpy as np

def callback(msg):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    range_min = 10.0
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < range_min and msg.ranges[i] != 0.0:
            range_min = msg.ranges[i]
            range_min_index = i
    print('min:', range_min, 'index: ', range_min_index)

    twist = Twist()
    
    ## method 1
    if (range_min < 1.5 and range_min > 0.2):
        if range_min_index > 10 and range_min_index < 180:
            twist.angular.z = (range_min_index - 10)/180.0 * 2.0 + 0.5
        elif range_min_index > 180 and range_min_index < 350:
            twist.angular.z = (range_min_index - 350)/180.0 * 2.0 - 0.5
        elif range_min_index > 350 or range_min_index < 10:
            twist.linear.x = 0.21
    
    # method 2 (worse than method 1)
    # if (range_min < 1.5 and range_min > 0.2):
    #     if range_min_index > 0 and range_min_index < 180:
    #         twist.angular.z = range_min_index/180.0 * 2.0 + 0.5
    #         twist.linear.x = range_min / 1.5 * 0.11 + 0.1
    #     elif range_min_index > 180 and range_min_index < 360:
    #         twist.angular.z = (range_min_index - 360)/180.0 * 2.0 - 0.5
    #         twist.linear.x = range_min / 1.5 * 0.11 + 0.1
    
    ## method 3
    if (range_min < 1.5 and range_min > 0.2):
        x = -range_min * np.cos(range_min_index/180.0 * 3.14)  # x = -r*cos(theta) (in the world frame)
        y = -range_min * np.sin(range_min_index/180.0 * 3.14)  # y = -r*sin(theta) (in the world frame)
        k_ro = 0.1
        k_alpha = 0.4
        k_beta = 0.0
        alpha = range_min_index/180.0 * 3.14
        beta = -np.arctan2(y, x)
        rho = np.sqrt(x**2 + y**2)
        v = k_ro * rho
        w = k_alpha * alpha + k_beta * beta
        print('x:', x, 'y:', y, 'alpha:', alpha, 'beta:', beta, 'rho:', rho)
        print('v:', v, 'w:', w)
        if w > 2.5:
            w = 2.5
        elif w < -2.5:
            w = -2.5
        if v > 0.21:
            v = 0.21
        elif v < -0.21:
            v = -0.21
        twist.linear.x = v
        twist.angular.z = w

        
    print('linear.x:', twist.linear.x, 'angular.z:', twist.angular.z)
    pub.publish(twist)


def pole_follower():

    rospy.init_node('pole_follower', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    pole_follower()
