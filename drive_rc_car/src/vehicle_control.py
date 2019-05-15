#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from Adafruit_PCA9685 import PCA9685
from Servo import ServoMotor
from std_msgs.msg import Float32MultiArray as TwoFloat32

#from DC_Motor import DC_Motor
#from std_msgs.msg import Int8
#from std_msgs.msg import String

class VehicleControl(object):
    def __init__(self):
        rospy.init_node('vehicle_controller')
        self.dc_pwm_pub = rospy.Publisher('dc_pwm',TwoFloat32,queue_size=2)
        self.pca = self._get_pca()
        self.servo = ServoMotor(pca)

        self.mode = 'standstill'

        self.pwm1 = 0
        self.pwm2 = 0

        self.loop_rate = rospy.Rate(10)


    def start(self):
        while not rospy.is_shutdown():
            self.emergency_listener()
            self.remote_controller_listener()
            self.loop_rate.sleep()

    def emergency_callback(data):
        self.pwm1 = 0
        self.pwm2 = 0
        self.publish_dc_pwm()

    def emergency_listener(self):
        rospy.Subscriber("emergency_stop", String, emergency_callback)

    def remote_controller_listener(self):
        rospy.Subscriber("remote",TwoFloat32,remote_controller_callback)

    def _get_pca(self):
        pca = PCA9685(busnum=1)            # check which bus that is used
        pca.set_pwm_freq(pwm_freq)
        print(pca)
        return pca

    def remote_controller_callback(self,data):
        speed_inc = data.data[0]
        steering_inc = data.data[0]
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
                self.pwm1 += inc
            elif inc < 0:
                self.mode = 'reverse'
                self.pwm2 += -1*inc

        elif self.mode == 'forward':
            if inc > 0:
                self.pwm1 += inc
            elif inc < 0:
                self.pwm1 += inc
                if self.pwm1 <= 0:
                    self.mode = 'standstill'
                    self.pwm1 = 0

        elif self.mode == 'reverse':
            if inc < 0:
                self.pwm2 += -1*inc
            elif inc > 0:
                self.pwm2 += -1*inc
                if self.pwm2 <= 0:
                    self.mode = 'standstill'
                    self.pwm2 = 0

        self.pwm1 = self.max_min_pwm(self.pwm1)
        self.pwm2 = self.max_min_pwm(self.pwm2)

        self.publish_dc_pwm()

    def publish_dc_pwm(self):
        msg = TwoFloat32()
        msg.data = [self.pwm1,self.pwm2]
        self.dc_pwm_pub(msg)

    def set_steering(self,inc):
        # TODO: fixes :)
        steering_pwm = inc + self.servo.get_current_pwm()
        self.servo.set_pwm(steering_pwm)
        print(steering_pwm)


if __name__ == '__main__':
    vc = VehicleControl()
    vc.start()
