#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from Adafruit_PCA9685 import PCA9685
from Servo import ServoMotor
from drive_rc_car.msg import ultrasonic
from drive_rc_car.msg import TwoFloat32
from std_msgs.msg import Int8
from std_msgs.msg import String


class Ultrasonic(object):
    def __init__(self):
        rospy.init_node('ultrasonic_node')
        # self.frontone = 1
        # More nodes will be defined here if necessary
        self.ultrasonic_pub = rospy.Publisher('ultrasonic_pub',ultrasonic,queue_size=2) 
        # ultrasonic sensing data published from the node
        #self.mode = 'standstill'
        #self.pwm = 0


    def start(self):
        self.ultrasonic_listener()
        rospy.spin()

    def ultrasonic_listener(self):
        rospy.Subscriber("ultrasonic", ultrasonic, self.ultrasonic_callback)   

    def ultrasonic_callback(self,data):
        self.frontone = data.sensor1
        # more sensors will be added here

        # end
        self.safetycheck(data)

    def safetycheck(self,data):
        # Something with safety check for the ultrasonic sensors will be added here

        # end


if __name__ == '__main__':
    ultra = Ultrasonic()
    ultra.start()
