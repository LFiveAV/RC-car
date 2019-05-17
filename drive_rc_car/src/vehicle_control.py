#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from Adafruit_PCA9685 import PCA9685
from Servo import ServoMotor
from drive_rc_car.msg import dc_pwm
from drive_rc_car.msg import TwoFloat32

#from DC_Motor import DC_Motor
from std_msgs.msg import Int8

from std_msgs.msg import String

MIN_PWM = 0
MAX_PWM = 1023

class VehicleControl(object):
    def __init__(self):
        rospy.init_node('vehicle_controller')
        self.dc_pwm_pub = rospy.Publisher('dc_pwm',dc_pwm,queue_size=2)
        self.pca = self._get_pca()
        self.servo = ServoMotor(self.pca)

        self.mode = 'standstill'

        self.pwm = 0
        self.steering_pwm = 0



    def start(self):
        self.emergency_listener()
        self.remote_controller_listener()
        rospy.spin()

    def emergency_callback(self,data):
        self.pwm = 0
        self.mode = 'standstill'
        self.publish_dc_pwm()

    def emergency_listener(self):
        rospy.Subscriber("emergency_stop", String, self.emergency_callback)

    def remote_controller_listener(self):
        rospy.Subscriber("remote",TwoFloat32,self.remote_controller_callback)

    def _get_pca(self):
        pwm_freq = 324 # FIX THIS
        pca = PCA9685(busnum=1)            # check which bus that is used
        pca.set_pwm_freq(pwm_freq)
        print(pca)
        return pca

    def remote_controller_callback(self,data):
        speed_inc = data.data1
        steering_inc = data.data2
        if speed_inc != 0:
            self.set_pwm(speed_inc)
        elif steering_inc != 0:
            self.set_steering(steering_pwm)

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

        self.publish_dc_pwm()

    def publish_dc_pwm(self):
        msg = dc_pwm()
        msg.pwm = self.pwm
        msg.mode = self.mode
        self.dc_pwm_pub.publish(msg)
        print self.mode,self.pwm

    def set_steering(self,inc):
        self.steering_pwm = inc + self.servo.get_current_pwm()
        self.servo.set_pwm(self.steering_pwm)
        print(self.steering_pwm)


if __name__ == '__main__':
    vc = VehicleControl()
    vc.start()
