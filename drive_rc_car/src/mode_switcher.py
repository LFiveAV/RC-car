#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray as TwoFloat32

class ModeSwitcher(object):
    def __init__(self):
        rospy.init_node('mode_switcher')
        self.pub = rospy.Publisher('control_input',TwoFloat32,queue_size=2)
        self.mode = 'remote' # self.mode must be 'remote' or 'self_drive'
        self.speed_inc = 0
        self.steering_inc = 0
        self.loop_rate = rospy.Rate(10)

    def start(self):
        while not rospy.is_shutdown():
            self.mode_listener()
            self.control_listener()
            self.pub_control()
            self.loop_rate.sleep()

    def mode_listener():
        rospy.Subscriber("mode",String,mode_callback)

    def mode_callback(self,data):
        self.mode = data.data

    def control_listener(self):
        rospy.Subscriber(self.mode,TwoFloat32,control_callback)

    def control_callback(self,data):
        self.speed_inc = data.data[0]
        self.steering_inc = data.data[1]

    def pub_control(self):
        msg = TwoFloat32()
        msg.data = [self.speed_inc,self.steering_inc]
        self.pub(msg)
        self.reset_inc()

    def reset_inc(self):
        self.speed_inc = 0
        self.steering_inc = 0


if __name__ == '__main__':
    ms = ModeSwitcher()
    ms.start()
