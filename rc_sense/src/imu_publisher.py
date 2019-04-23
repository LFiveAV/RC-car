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

from Adafruit_BNO055 import BNO055
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

pub_rate = 1


def publish_imu_data():
    imu_msg = Imu()
    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=5)       # Queue size??
    # rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(pub_rate) # 10hz

    while not rospy.is_shutdown():

        acc_x, acc_y, acc_z = bno.read_linear_acceleration()
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z

        gyr_x, gyr_y, gyr_z = bno.read_gyroscope()        # gives values in degrees per second
        imu_msg.angular_velocity.x = (math.pi/180)*gyr_x
        imu_msg.angular_velocity.y = (math.pi/180)*gyr_y
        imu_msg.angular_velocity.z = (math.pi/180)*gyr_z
        x,y,z,w = bno.read_quaternion()
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation.w = w

        imu_pub.publish(imu_msg)
        rate.sleep()


def publish_imu_pub_rate(pub_rate):
    pub = rospy.Publisher('imu_msg_rate', Int8, latch=True)
    rate_msg = Int8()
    rate_msg.data = pub_rate
    pub.publish(rate_msg)


if __name__ == '__main__':
    rospy.init_node('imu_node', anonymous=True)
    print("in main")
     # Initialize imu object
    bno = BNO055.BNO055(busnum = 0)
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')


    # Print system status and self test result.
    status,self_test,error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    print('imu publisher node successfully started')

    try:
        publish_imu_pub_rate(pub_rate)
        print("published rate")
        publish_imu_data()
    except rospy.ROSInterruptException:
        pass
