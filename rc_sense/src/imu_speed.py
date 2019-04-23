#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 18 16:32:49 2018

@author: nvidia
"""

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8


global imu_msg_rate
imu_msg_rate = 0

global v_x, v_y, v_z


def imu_callback(data):
    if imu_msg_rate == 0:
        print("Cannot calculate speed from IMU data. No time interval given.")
    else:
        global v_x, v_y, v_z
        imu_T = 1/imu_msg_rate
        v_x =+ imu_T*data.linear_acceleration.x
        v_y =+ imu_T*data.linear_acceleration.y
        v_z =+ imu_T*data.linear_acceleration.z


def imu_listener():
    rospy.Subscriber("imu_data", Imu, imu_callback)


def rate_callback(data):
    global imu_msg_rate
    imu_msg_rate = data.data


def rate_listener():
    rospy.Subscriber("imu_msg_rate", Int8, rate_callback)


def imu_speed_publisher():
    imu_speed = Vector3()
    pub = rospy.Publisher('imu_speed', Vector3, queue_size=2)
    rate = rospy.Rate(1)
    global v_x, v_y, v_z
    while not rospy.is_shutdown():
        imu_speed.x = v_x
        imu_speed.y = v_y
        imu_speed.z = v_z
        pub.publish(imu_speed)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('imu_listener', anonymous=True)
    print("[Node] imu_speed started")
    v_x = 0
    v_y = 0
    v_z = 0

    rate_listener()

    imu_listener()

    try:
        imu_speed_publisher()
    except rospy.ROSInterruptException:
        pass