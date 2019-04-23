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
import math


from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int8

heading_des = 0
global lateral_dist_des
global lat_k_p

pub = rospy.Publisher("steering_pwm", Int8, queue_size=1)


def p_controller(lat_dev, lat_k_p):
    lat_control = lat_k_p * lat_dev
    return lat_control


def callback(wall_dist):
    dist = wall_dist.y
    heading = wall_dist.theta
    lat_dev = lateral_dist_des - dist
    yaw_dev = heading_des - heading
    print(lat_dev)
    # print(yaw_dev)
    steering_control = p_controller(lat_dev, lat_k_p)

    pub.publish(steering_control)


def wall_dist_listener():
    rospy.Subscriber("wall_distance", Pose2D, callback)

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    print('[Node] wall_follower started')
    lat_k_p = -1
    lateral_dist_des = 0.15
    wall_dist_listener()
    rospy.spin()