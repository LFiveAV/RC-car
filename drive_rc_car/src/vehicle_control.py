#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from drive_rc_car.msg import control
from drive_rc_car.msg import TwoFloat32
from std_msgs.msg import Int8
from std_msgs.msg import String

MIN_PWM = 0
MAX_PWM = 1023

class VehicleControl(object):
    def __init__(self):
        self.pwm_pub = rospy.Publisher('pwm_node',control,queue_size=2)

        self.mode = 'standstill'

        self.pwm = 0
        self.servo_pwm = 100



    def start(self):
        self.emergency_listener()
        self.remote_controller_listener()
        rospy.spin()

    def emergency_callback(self,data):
        self.pwm = 0
        self.mode = 'standstill'
        self.publish_pwm()

    def emergency_listener(self):
        rospy.Subscriber("emergency_stop", String, self.emergency_callback)

    def remote_controller_listener(self):
        rospy.Subscriber("control_input",TwoFloat32,self.remote_controller_callback)

    def remote_controller_callback(self,data):
        speed_inc = data.data1
        steering_inc = data.data2
        if speed_inc != 0:
            self.set_pwm(speed_inc)
        elif steering_inc != 0:
            self.set_steering(steering_inc)
        self.publish_pwm()

        #speed_thread = threading.Thread(name='speed',target=self.set_pwm,args=(speed_inc,))
        #steering_thread = threading.Thread(name='steering',target=self.set_steering, args=(steering_inc,))
        #speed_thread.start()
        #steering_thread.start()
        #speed_thread.join(0.3) # Maybe remove 1
        #steering_thread.join(0.3) # Maybe remove 1

    def max_min_pwm(self,pwm):
        return min(MAX_PWM,max(MIN_PWM,pwm))

    def set_pwm(self,inc):
        if self.mode == 'standstill':
            if inc > 0:
                self.mode = 'forward'
                self.pwm += inc
            elif inc < 0:
                self.mode = 'reverse'
                self.pwm += -1*inc

        elif self.mode == 'forward':
            if inc > 0:
                self.pwm += inc
            elif inc < 0:
                self.pwm += inc
                if self.pwm <= 0:
                    self.mode = 'standstill'
                    self.pwm = 0

        elif self.mode == 'reverse':
            if inc < 0:
                self.pwm += -1*inc
            elif inc > 0:
                self.pwm += -1*inc
                if self.pwm <= 0:
                    self.mode = 'standstill'
                    self.pwm = 0

        self.pwm = self.max_min_pwm(self.pwm)



    def publish_pwm(self):
        msg = control()
        msg.motor_pwm = self.pwm
        msg.mode = self.mode
        msg.servo_pwm = self.servo_pwm
        self.pwm_pub.publish(msg)
        print self.mode,self.pwm,self.servo_pwm

    def set_steering(self,inc):
        self.servo_pwm += inc
        self.servo_pwm = max(55,min(self.servo_pwm,145))




if __name__ == '__main__':
    rospy.init_node('vehicle_controller')
    vc = VehicleControl()
    vc.start()
