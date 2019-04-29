#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:35:35 2018

@author: nvidia
"""

import rospy, yaml

from Adafruit_PCA9685 import PCA9685
from Servo import ServoMotor
from DC_Motor import DC_Motor
from std_msgs.msg import Int8
from std_msgs.msg import String

global servo, dc_motor, pca


path = "../config/settings.yaml"
settings = yaml.safe_load(open(path))
dc_motor_pwm = settings['dc_motor']
servo_pwm = settings['servo']

pwm_freq = 50

# DC-motor
STANDSTILL_PWM = dc_motor_pwm['pwm_midpoint']
pwm_min = dc_motor_pwm['pwm_min']
pwm_max = dc_motor_pwm['pwm_max']
pwm_midrange_min = dc_motor_pwm['pwm_midrange_min']
pwm_midrange_max = dc_motor_pwm['pwm_midrange_max']
dc_init_pwm = STANDSTILL_PWM
mode = 'neutral'

# Servo
servo_init_pwm = servo_pwm['pwm_midpoint']

def get_reverse_pwm():
    return (pwm_min, pwm_midrange_min-1)

def get_neutral_pwm():
    return (pwm_midrange_min, pwm_midrange_max)

def get_forward_pwm():
    return (pwm_midrange_max+1, pwm_max)

def in_range(_range,pwm_val):
    return pwm_val >= _range[0] and pwm_val <= _range[1]

def get_mode(pwm_val):
    if in_range(get_reverse_pwm(),pwm_val):
        return 'reverse'
    elif in_range(get_forward_pwm(),pwm_val):
        return 'forward'
    else:
        return 'neutral'

def dc_callback(data):
    inc = data.data
    speed_pwm = inc + dc_motor.get_current_pwm()
    if mode is 'neutral' and inc>0:
        speed_pwm = max(get_forward_pwm()[0],speed_pwm)
        mode = get_mode(speed_pwm)
    elif mode is 'neutral' and inc<0:
        speed_pwm = min(get_reverse_pwm()[1],speed_pwm)
        mode = get_mode(speed_pwm)
    elif mode is 'forward' and speed_pwm<get_forward_pwm()[0]:
        speed_pwm = dc_init_pwm
        mode = 'neutral'
    elif mode is 'reverse' and speed_pwm>get_reverse_pwm()[1]:
        speed_pwm = dc_init_pwm
        mode = 'neutral'
    dc_motor.set_pwm(speed_pwm)
    print(mode,speed_pwm)


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

    pca = PCA9685(busnum=1)            # check which bus that is used
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
