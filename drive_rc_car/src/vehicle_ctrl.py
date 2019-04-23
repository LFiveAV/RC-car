#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:35:35 2018

@author: nvidia
"""

import rospy

from Adafruit_PCA9685 import PCA9685
from Servo import ServoMotor
from DC_Motor import DC_Motor
from std_msgs.msg import Int8
from std_msgs.msg import String

global servo, dc_motor, pca

STANDSTILL_PWM = 315

pwm_freq = 50
dc_init_pwm = 315
servo_init_pwm  =310


def dc_callback(data):
    speed_pwm = data.data + dc_motor.get_current_pwm()
    dc_motor.set_pwm(speed_pwm)
    print(speed_pwm)

def speed_listener():
    rospy.Subscriber("speed_pwm", Int8, dc_callback)
    
def servo_callback(data):
    steering_pwm = data.data + servo.get_current_pwm()
    servo.set_pwm(steering_pwm) 
    print(steering_pwm)

def steering_listener():
    rospy.Subscriber("steering_pwm", Int8, servo_callback)
    
def emergency_callback(data):
    dc_motor.set_pwm(STANDSTILL_PWM)

def emergency_listener():
    rospy.Subscriber("emergency_stop", String, emergency_callback)



if __name__ == '__main__':
    rospy.init_node('vehicle_controller')       
    
    
    pca = PCA9685(busnum=1)
    pca.set_pwm_freq(pwm_freq)
    print(pca)


    servo = ServoMotor(pca)
    dc_motor = DC_Motor(pca)
    
    emergency_listener()    
    
    servo.set_pwm(servo_init_pwm)
    dc_motor.set_pwm(dc_init_pwm)
    
    speed_listener()
    steering_listener()
    rospy.spin()