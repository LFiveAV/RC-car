#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String
from std_msgs.msg import Int8
from drive_rc_car.msg import TwoFloat32


class ModeSwitcher(object):
    def __init__(self):
        
        self.pub = rospy.Publisher('control_input',TwoFloat32,queue_size=2)
        self.mode = 'remote' # self.mode must be 'remote' or 'self_drive'
        self.speed_inc = 0
        self.steering_inc = 0
        self.rate = rospy.Rate(10)

    def start(self):
        while not rospy.is_shutdown():
            self.mode_listener()
            if self.mode == 'remote':
                self.remote_listener()
            else:
                self.self_drive_listener()
            
            self.rate.sleep()

    def mode_listener(self):
        rospy.Subscriber("mode",String,self.mode_callback)

    def mode_callback(self,data):
        if not self.mode == data.data:
            self.mode = data.data
            print (self.mode)

    def remote_listener(self):
        rospy.Subscriber('speed_pwm',Int8,self.speed_callback)
        rospy.Subscriber('steering_pwm',Int8,self.steering_callback)

        self.pub_control()

    def self_drive_listener(self):
        rospy.Subscriber('speed_pwm_sd',Int8,self.speed_callback)
        rospy.Subscriber('steering_pwm_sd',Int8,self.steering_callback)

        self.pub_control()

    def speed_callback(self,data):
        self.speed_inc = data.data

    def steering_callback(self,data):
        self.steering_inc = data.data

    def pub_control(self):
        msg = TwoFloat32()
        msg.data1 = self.speed_inc
        msg.data2 = self.steering_inc
        self.pub.publish(msg)
        self.reset_inc()

    def reset_inc(self):
        self.speed_inc = 0
        self.steering_inc = 0


if __name__ == '__main__':
    rospy.init_node('mode_switcher')
    ms = ModeSwitcher()
    ms.start()
