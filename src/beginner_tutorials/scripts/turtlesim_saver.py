#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class vel_manipulator:
    
    def __init__(self):
        pub_topic_name ="/turtle1/cmd_vel"
        sub_topic_name ="/turtle1/pose"

        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Pose, self.pose_callback)
        self.velocity_msg = Twist()

    def pose_callback(self, msg):
        if (msg.x >= 10 or msg.x <= 5 or msg.y >= 10 or msg.y <= 5):
            self.velocity_msg.linear.x = -1.0
            self.pub.publish(self.velocity_msg)
        # else:
            # self.velocity_msg.linear.x = 0

        


if __name__ == '__main__':
    node_name = "Turtlesim_Saver"
    rospy.init_node(node_name)
    vel_manipulator()
    rospy.spin()