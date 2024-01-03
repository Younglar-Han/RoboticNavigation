#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys

NAV = 1
TRACK = 0
nav_goal = 0  # the goal point
sys_state = NAV
lidar_update_time = 0

def move_to_goal(point):
    # 创建一个action client来与move_base服务器交互
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 等待服务器启动
    client.wait_for_server()

    # 定义目标
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置
    goal.target_pose.pose.position.x = point[0]
    goal.target_pose.pose.position.y = point[1]
    goal.target_pose.pose.position.z = 0.0
    
    theta = point[2] * 3.1415926 / 180.0
    goal.target_pose.pose.orientation.w = np.cos(theta / 2.0)
    goal.target_pose.pose.orientation.x = np.sin(theta / 2.0)
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0


    # 发送目标
    client.send_goal(goal)

    # 等待结果
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def nav_pose():
    global nav_goal
    global sys_state
    global lidar_update_time
        
    points = np.zeros((7,3))
    points[0] = [-3.963, -3.517, 0.0] # P2
    points[1] = [-2.980, -2.506, 0.0] # P2 center
    points[2] = [0.039, -3.653, 0.0] # P3
    points[3] = [-0.760, -3.548, 0.0] # P3 center
    points[4] = [0.133, 0.359, 0.0] # P4
    points[5] = [0.016, 0.00, 0.0] # P4 center
    points[6] = [-3.800, 0.434, 0.0] # P1
    print('naving')
    if sys_state == NAV:
        try:
            result = move_to_goal(points[nav_goal])
            if result:
                nav_goal += 1
                if nav_goal % 2 == 0:
                    sys_state = TRACK
                    lidar_update_time = 0
            if nav_goal >= len(points):
                print("END")
                rospy.signal_shutdown("END")
        except rospy.ROSInterruptException:
            pass
    
def pole_track(msg):
    global nav_goal
    global sys_state
    global lidar_update_time
    
    if sys_state == TRACK:
        if lidar_update_time < 15:
            lidar_update_time += 1
            print('lidar_update_time:', lidar_update_time, end='\r')
            return
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        range_min = 10.0
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < range_min and msg.ranges[i] != 0.0:
                range_min = msg.ranges[i]
                range_min_index = i
                range_min_angle = msg.angle_min + i * msg.angle_increment
                range_min_angle = range_min_angle * 180.0 / 3.1415926
                if range_min_angle > 180:
                    range_min_angle = range_min_angle - 360  # -180 ~ 180
        print('min:', range_min, 'index: ', range_min_index)

        twist = Twist()
        
        range_threshold = 0.2
        angle_threshold = 10
        twist = Twist()
        if range_min > range_threshold:
            if abs(range_min_angle) < angle_threshold:
                twist.linear.x = 0.21
        if abs(range_min_angle) > angle_threshold:
            twist.angular.z = range_min_angle/180.0 * 2.0 + np.sign(range_min_angle) * 0.5
        
        print('linear.x:', twist.linear.x, 'angular.z:', twist.angular.z)
        pub.publish(twist)
        if range_min < range_threshold and abs(range_min_angle) < angle_threshold:
            sys_state = NAV
            print('Arrived at the pole!')
            time.sleep(1.0)
    else: 
        nav_pose()
    pass

def nav_track():

    rospy.init_node('nav_track', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, pole_track)

    rospy.spin()

if __name__ == '__main__':
    nav_track()