#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:29:50 2018

@author: nvidia
"""

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3

global rc_controller
global rate


class RC_Controller(object):

    def __init__(self, kp_sp=2, kp_ang=0.5):
        self.act_speed = 0
        self.act_steering_angle = 0
        self.des_speed = 0
        self.des_steering_angle = 0
        self.Kp_speed = kp_sp
        self.Kp_angle = kp_ang
        self.pub_steering = rospy.Publisher('steering_pwm', Int8, queue_size=1)
        self.pub_speed = rospy.Publisher('speed_pwm', Int8, queue_size=1)

    def speed_control(self):
        err = self.des_speed - self.act_speed
        print "error: ", err
        d_pwm = self.Kp_speed * err
        d_pwm = int(round(d_pwm))
        print 'speed control:', d_pwm
        self.pub_speed.publish(d_pwm)
        #self.publish_speed_pwm(d_pwm)

    def steering_control(self):
        err = self.des_steering_angle - self.act_steering_angle
        d_pwm = self.Kp_angle * err
        d_pwm = int(round(d_pwm))
        print("steering control")
        print(d_pwm)
        self.pub_steering.publish(d_pwm)

    def publish_speed_pwm(self, pwm):
        self.pub_speed.publish(pwm)


def speed_callback(msg):
    rc_controller.act_speed = msg.x
    rc_controller.speed_control()


def angle_callback(msg):
    rc_controller.act_steering_angle = msg.data


def steering_command_callback(msg):
    rc_controller.des_steering_angle = msg.data
    print("steering_command_callback")
    rc_controller.steering_control()


def speed_command_callback(msg):
    rc_controller.des_speed = msg.data
    rc_controller.speed_control()


def speed_listener():
    rospy.Subscriber("imu_speed", Vector3, speed_callback)      # only imu atm


def angle_listener():
    rospy.Subscriber("vehicle_steering_angle", Float32, angle_callback)


def steering_command_listener():
    rospy.Subscriber("steering_command", Float32, steering_command_callback)


def speed_command_listener():
    rospy.Subscriber("speed_command", Float32, speed_command_callback)


if __name__ == '__main__':
    rospy.init_node('drive_controller_node')
    rate = rospy.Rate(10)
    print("[Node] drive_controller started")

    rc_controller = RC_Controller()

    speed_listener()
    angle_listener()
    speed_command_listener()
    steering_command_listener()

    rospy.spin()


