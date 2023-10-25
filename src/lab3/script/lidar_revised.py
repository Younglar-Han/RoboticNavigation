 #! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
scann = LaserScan()
real_range_max_golbal = 0.0
real_range_min_global = 10.0
sample_point_max = 0.0
sample_point_min = 10.0
sample_point_sum = 0.0
sample_point_mean = 0.0
sample_point_num = 0

def callback(msg):
    global real_range_max_golbal
    global real_range_min_global
    global sample_point_max
    global sample_point_min
    global sample_point_sum
    global sample_point_mean
    global sample_point_num
    current_time = rospy.Time.now()
    scann.header.stamp = current_time
    scann.header.frame_id = 'laser'
    scann.angle_min = 0.0
    scann.angle_max = 6.28318458203
    scann.angle_increment = 0.0174532923847
    scann.time_increment = 2.98899994959e-05
    scann.scan_time = msg.scan_time
    scann.range_min = 0.119999997318
    scann.range_max = 3.5
    scann.ranges = msg.ranges[180]
    scann.intensities = msg.intensities
    
    # detect the min and max range #
    real_range_max = max(msg.ranges)
    real_range_min = 10.0
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < real_range_min and msg.ranges[i] > 0.01:
            real_range_min = msg.ranges[i]
    if real_range_max > real_range_max_golbal:
        real_range_max_golbal = real_range_max
    if real_range_min < real_range_min_global:
        real_range_min_global = real_range_min
    print('real range max =', real_range_max_golbal, '  real range min =', real_range_min_global)
    
    # compute the precision and accuracy #
    if msg.ranges[0] > sample_point_max and msg.ranges[0] != 0.0:
        sample_point_max = msg.ranges[0]
    if msg.ranges[0] < sample_point_min and msg.ranges[0] != 0.0:
        sample_point_min = msg.ranges[0]
    if msg.ranges[0] != 0.0:
        sample_point_sum += msg.ranges[0]
        sample_point_num += 1
        sample_point_mean = sample_point_sum / sample_point_num
    
    print('sample point max =', sample_point_max, '  sample point min =', sample_point_min)
    print('sample point mean =', sample_point_mean)
    print('precision =', sample_point_max - sample_point_min)
    # real range max = 4.196000099182129   real range min = 0.09099999815225601
    # sample point max = 0.5199999809265137   sample point min = 0.5180000066757202
    # sample point mean = 0.5191752492767019   real = 0.5100
    # precision = 0.001999974250793457
    
    # real range max = 4.197999954223633   real range min = 0.09099999815225601
    # sample point max = 1.6579999923706055   sample point min = 1.6250
    # sample point mean = 1.6388787898150357   real = 1.652
    # precision = 0.03299999237060547
    
    # real range max = 4.186999797821045   real range min = 0.09099999815225601
    # sample point max = 0.29100000858306885   sample point min = 0.28999999165534973
    # sample point mean = 0.29022555938340666   real = 0.2861
    # precision = 0.0010000169277191162



def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()