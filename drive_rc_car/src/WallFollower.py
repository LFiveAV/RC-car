#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 18 11:57:33 2018

@author: nvidia
"""

import rospy
import logging
import sys
import time


from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int8


class WallFollower(object):

    def __init__(self, desired_dist):
        self.pub = rospy.Publisher("steering_pwm", Int8, queue_size=1)
        self.lat_kp = 5
        self.yaw_kp = 2
        self.lateral_dist_des = desired_dist
        self.heading_des = 0
        self.wall_dist = 0
        self.prev_dist = 1
        self.counter = 0
        self.wall_dist_listener()

    def p_controller(self, lat_dev):
        lat_control = self.lat_kp * lat_dev
        return lat_control

    def callback(self, dist_msg):
        dist = dist_msg.y
        heading = dist_msg.theta
        lat_dev = self.lateral_dist_des - dist
        yaw_dev = self.heading_des - heading
        print(lat_dev)
        # print(yaw_dev)
        steering_control = self.p_controller(lat_dev)
        self.pub.publish(steering_control)

    def wall_dist_listener(self):
        rospy.Subscriber("wall_distance", Pose2D, self.callback)


if __name__ == '__main__':
    rospy.init_node("WallFollower")
    print('[Node] WallFollower started')
    wall_follower = WallFollower(0.15)
    rospy.spin()