#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y):
    # 创建一个action client来与move_base服务器交互
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 等待服务器启动
    client.wait_for_server()

    # 定义目标
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    # 发送目标
    client.send_goal(goal)

    # 等待结果
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtlebot_client')

        result = move_to_goal(-3.963, -3.517) # 第二个点
        result = move_to_goal(0.039, -3.653) # 第三个点
        result = move_to_goal(0.133, 0.359) # 第四个点
        result = move_to_goal(-3.800, 0.434) # 第一个点
    except rospy.ROSInterruptException:
        pass