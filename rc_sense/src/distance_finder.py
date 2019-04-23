#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 18 11:57:33 2018

@author: nvidia
"""

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D


dist_pub = rospy.Publisher("wall_distance", Pose2D, queue_size=3)


def get_range(theta, scan_data):
    index_0 = math.floor(-scan_data.angle_min / scan_data.angle_increment)
    index = int(math.floor(theta / scan_data.angle_increment) + index_0)
    range_scan = scan_data.ranges[index]
    if range_scan < scan_data.range_min or math.isnan(range_scan):
        range_scan = 0
    elif range_scan > scan_data.range_max or math.isinf(range_scan):
        range_scan = scan_data.range_max
    return range_scan


def get_range_from_index(index, scan_data):
    range_scan = scan_data.ranges[index]
    if range_scan < scan_data.range_min or math.isnan(range_scan):
        range_scan = 0
    elif range_scan > scan_data.range_max or math.isinf(range_scan):
        range_scan = scan_data.range_max
    return range_scan


def get_averaged_range(theta, scan_data, f_window):
    scan_ranges = []
    index_0 = math.floor(-scan_data.angle_min / scan_data.angle_increment)
    index = int(math.floor(theta / scan_data.angle_increment) + index_0)
    for ind in range(index - f_window, index + f_window + 1):
        scan_ranges.append(get_range_from_index(ind, scan_data))
        median_range = np.median(scan_ranges)
    return median_range


def get_distance_and_heading(theta_a, theta_b, range_a, range_b):
    angle_diff = theta_a - theta_b
    phi = math.atan2(
        (range_a * math.cos(angle_diff) - range_b),
        (range_a * math.sin(angle_diff)))
    distance_to_wall = range_b * math.cos(phi)
    return distance_to_wall, phi


def lidar_callback(laser_scan):
    theta_a = math.radians(-30)
    range_a = get_averaged_range(theta_a, laser_scan, 3)
    theta_b = math.radians(-90)
    range_b = get_averaged_range(theta_b, laser_scan, 3)
    print(range_b)

    distance, phi = get_distance_and_heading(theta_a, theta_b, range_a, range_b)

    dist_msg = Pose2D()
    dist_msg.y = distance
    dist_msg.theta = phi

    dist_pub.publish(dist_msg)


def lidar_listener():
    rospy.Subscriber("scan", LaserScan, lidar_callback)


if __name__ == '__main__':
    rospy.init_node('distance_finder', anonymous=True)
    print('[Node] distance_finder started')
    rospy.Subscriber("scan", LaserScan, lidar_callback)
    rospy.spin()
