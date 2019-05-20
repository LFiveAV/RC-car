#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import threading

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

THETA_RIGHT = (-30,-90)
THETA_LEFT = (0,0)

class DistanceFinder(object):
    def __init__(self):
        rospy.init_node('lidar_wall_distance_finder', anonymous=True)
        print('[Node] distance_finder started')
        self.dist_pub = rospy.Publisher("liadr_wall_distance",Point,queue_size=3)
        self.left = -1
        self.right = -1

    def start(self):
        self.lidar_listener()
        rospy.spin()

    def get_distance(self,theta,laser_data,side):
        theta_a = math.radians(theta[0])
        theta_b = math.radians(theta[1])
        range_a = get_averaged_range(theta_a, laser_scan, 3)
        range_b = get_averaged_range(theta_b, laser_scan, 3)

        distance, phi = get_distance_and_heading(theta_a, theta_b,
                                                 range_a, range_b)
        # TODO: Publish phi. New message maybe?
        if side == 'right':
            self.right = distance
        elif side == 'left':
            self.left = distance

    def lidar_callback(self,laser_data):
        right_thread = threading.Thread(name='right',target=self.get_distance,
                                        args=(THETA_RIGHT, laser_data,'right',))
        right_thread = threading.Thread(name='left',target=self.get_distance,
                                        args=(THETA_LEFT, laser_data, 'left',))
        right_thread.start()
        left_thread.start()
        right_thread.join(0.5)
        left_thread.join(0.5)

        self.publish_distance()

    def publish_distance(self):
        dist_msg = Point()
        dist_msg.x = self.right
        dist_msg.y = self.left
        self.dist_pub.publish(dist_msg)
        self.right = -1
        self.left = -1

    def lidar_listener(self):
        rospy.Subscriber("scan", LaserScan, self.lidar_callback)

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

if __name__ == '__main__':
    df = DistanceFinder()
    df.start()
