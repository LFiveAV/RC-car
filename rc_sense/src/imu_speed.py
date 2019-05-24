#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8

class ImuSpeed(object):
    def __init__(self):
        rospy.init_node('imu_listener', anonymous=True)
        self.pub = rospy.Publisher('imu_speed', Vector3, queue_size=2)
        self.reset_v()
        self.imu_msg_rate = 0

    def start(self):
        self.rate_listener()
        self.imu_listener()
        self.publish_imu_speed()
        rospy.spin()

    def imu_callback(self,data):
        if self.imu_msg_rate == 0:
            print("Cannot calculate speed from IMU data. No time interval given.")
        else:
            imu_T = 1/self.imu_msg_rate
            self.v_x = imu_T*data.linear_acceleration.x
            self.v_y = imu_T*data.linear_acceleration.y
            self.v_z = imu_T*data.linear_acceleration.z

    def imu_listener(self):
        rospy.Subscriber("imu_data", Imu, imu_callback)

    def rate_callback(self,data):
        self.imu_msg_rate = data.data

    def rate_listener(self):
        rospy.Subscriber("imu_msg_rate", Int8, rate_callback)

    def publish_imu_speed(self):
        ime_speed = Vector3()
        imu_speed.x = self.v_x
        imu_speed.y = self.v_y
        imu_speed.z = self.v_z
        self.pub.publish(imu_speed)
        self.reset_v()

    def reset_v(self):
        self.v_x = -1
        self.v_y = -1
        self.v_z = -1

if __name__ == '__main__':
    is = ImuSpeed()
    is.start()
