#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import tf
from PIL import Image

nav_goal = 0  # the goal point
nav_state = 1  # 0:not naving  1:naving
track_state = 0  # 0:not tracking  1:tracking
# 全局变量
listener = None  # tf监听器
map_data = None  # 存储地图数据

# 墙的阈值
wall_threshold = 50

map_file = "docs/map.pgm"  # 修改为你的地图文件路径

def load_map(map_file):
    """
    从pgm文件加载地图，并将其转换为numpy数组。
    :param map_file: 地图文件的路径。
    :return: numpy数组形式的地图数据。
    """
    try:
        # 使用Pillow库加载地图图像
        map_image = Image.open(map_file)
        
        # 将图像转换为灰度格式（如果尚未是灰度图）
        map_image = map_image.convert('L')
        
        # 将图像数据转换为numpy数组
        map_data = np.array(map_image)
        
        return map_data
    except Exception as e:
        print("Failed to load map file:", e)
        return None

# 使用函数加载地图
map_data = load_map(map_file)

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

    points = np.zeros((4, 3))
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
    global nav_goal, nav_state, track_state, listener, map_data

    if track_state != 0:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        if listener is None or map_data is None:
            return

        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF Exception")
            return

        closest_point = None
        closest_distance = float('inf')

        for i, range in enumerate(msg.ranges):
            if range < msg.range_max:  # 检查距离是否有效
                # 计算激光点的角度和距离
                angle = msg.angle_min + i * msg.angle_increment
                laser_point = [range * np.cos(angle), range * np.sin(angle), 0.0]

                # 将激光点转换为地图坐标
                map_point = point_laser_to_map(laser_point, trans, rot)
                if not is_wall(map_point) and range < closest_distance:
                    closest_distance = range
                    closest_point = i  # 保存最近的非墙壁点的索引
                    print(closest_point)

        twist = Twist()

        ## method 1
        if (closest_point < 1.5 and closest_point > 0.2):
            if closest_point > 10 and closest_point < 180:
                twist.angular.z = (closest_point - 10) / 180.0 * 2.0 + 0.5
            elif closest_point > 180 and closest_point < 350:
                twist.angular.z = (closest_point - 350) / 180.0 * 2.0 - 0.5
            elif closest_point > 350 or closest_point < 10:
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


def point_laser_to_map(laser_point, trans, rot):
    # 这里暂且不考虑z轴，因为在二维地图中我们主要关注x和y。
    laser_x, laser_y, _ = laser_point
    trans_x, trans_y, _ = trans

    # 计算激光点相对于机器人的全局坐标
    # 旋转矩阵（仅在二维平面上旋转）
    theta = tf.transformations.euler_from_quaternion(rot)[2]  # 获取欧拉角的Z轴旋转分量
    global_x = trans_x + laser_x * np.cos(theta) - laser_y * np.sin(theta)
    global_y = trans_y + laser_x * np.sin(theta) + laser_y * np.cos(theta)

    # 将全局坐标转换为地图像素坐标（假设你已经知道地图的分辨率和原点位置）
    # 注意：这里需要根据你实际地图的元数据进行调整
    map_resolution = 0.05  # 地图分辨率，单位：米/像素
    origin_x, origin_y = -10, -10  # 地图原点在地图像素坐标系中的位置

    map_x = int((global_x / map_resolution) + origin_x)
    map_y = int((global_y / map_resolution) + origin_y)

    return map_x, map_y


def is_wall(map_point):
    """
    判断给定的地图坐标点是否为墙。
    :param map_point: 地图坐标系中的点，格式为(x, y)。
    :return: 布尔值，如果该点是墙则为True，否则为False。
    """
    global map_data
    x, y = map_point

    # 检查坐标是否在地图范围内
    if 0 <= x < map_data.shape[1] and 0 <= y < map_data.shape[0]:
        # 读取对应点的值（这里假设map_data已经是二维数组形式的地图数据）
        pixel_value = map_data[y][x]

        # 判断是否为墙，这里阈值需要根据你的地图进行调整
        # 通常地图中，较高的值表示被占据（可能是墙），较低的值表示自由空间
        return pixel_value > wall_threshold
    else:
        # 如果点不在地图上，返回False
        return False

def nav_track():
    rospy.init_node('nav_track', anonymous=True)
    # rospy.init_node('move_turtlebot_client')

    rospy.Subscriber('/scan', LaserScan, pole_track)

    rospy.spin()


if __name__ == '__main__':
    nav_track()