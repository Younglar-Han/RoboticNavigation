#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

nav_goal = 0  # the goal point
nav_state = 1  # 0:not naving  1:naving
track_state = 0  # 0:not tracking  1:tracking

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
    global nav_state
    global track_state
        
    points = np.zeros((4,3))
    points[0] = [-3.963, -3.517, 0.0]
    points[1] = [0.039, -3.653, 0.0]
    points[2] = [0.133, 0.359, 0.0]
    points[3] = [-3.800, 0.434, 0.0]
    print('naving')
    if nav_state != 0:
        try:
            result = move_to_goal(points[nav_goal])
            if result:
                nav_state = 0
                track_state = 1
        except rospy.ROSInterruptException:
            pass
    
def pole_track(msg):
    global nav_goal
    global nav_state
    global track_state

    if track_state != 0:
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
            print('linear.x:', twist.linear.x, 'angular.z:', twist.angular.z)
            pub.publish(twist)
        else:
            print('linear.x:', twist.linear.x, 'angular.z:', twist.angular.z)
            pub.publish(twist)
            nav_goal += 1
            nav_state = 1
            track_state = 0
    else: 
        nav_pose()
    pass

def nav_track():

    rospy.init_node('nav_track', anonymous=True)
    # rospy.init_node('move_turtlebot_client')

    rospy.Subscriber('/scan', LaserScan, pole_track)

    rospy.spin()

if __name__ == '__main__':
    nav_track()