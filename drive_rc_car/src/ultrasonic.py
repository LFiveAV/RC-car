#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from drive_rc_car.msg import ultrasonic
from drive_rc_car.msg import TwoFloat32
from drive_rc_car.msg import dc_pwm
from std_msgs.msg import Int8
from std_msgs.msg import String

D_LIM = 1000 # This is under discussion and will be edited later on 

class Ultrasonic(object):
    def __init__(self):
        rospy.init_node('ultrasonic_node')
        # self.frontone = 1
        # More nodes will be defined here if necessary
        self.ultrasonic_pub = rospy.Publisher('ultrasonic_pub',ultrasonic,queue_size=2) 
        self.emergency_pub = rospy.Publisher("emergency_stop", String, queue_size=2)
        self.mode = 'standstill'
        self.sensors = [-1 for i in range(8)]
        # ultrasonic sensing data published from the node



    def start(self):
        self.ultrasonic_listener()
        rospy.spin()

    def ultrasonic_listener(self):
        rospy.Subscriber("ultrasonic", ultrasonic, self.ultrasonic_callback)   

    def ultrasonic_callback(self,data):
        self.sensors[0] = data.sensor1 # front 1
        self.sensors[1] = data.sensor2 # front 2
        self.sensors[2] = data.sensor3 # front 3
        self.sensors[3] = data.sensor4 # front 4
        self.sensors[4] = data.sensor5 # rear 1 
        self.sensors[5] = data.sensor6 # rear 2
        self.sensors[6] = data.sensor7 # rear 3 
        self.sensors[7] = data.sensor8 # rear 4 
        rospy.Subscriber("dc_pwm",dc_pwm,self.dc_pwm_callback)
        self.safetycheck()
    def dc_pwm_callback(self,data):
        self.mode = data.mode
        
        
    def safetycheck(self):
        # Something with safety check for the ultrasonic sensors will be added here
        # Train in NN for obstacle avoidance
        # end
        rospy.Subscriber("dc_pwm",dc_pwm,self.dc_pwm_callback)
        for ultra_sensor in self.sensors[:int(len(self.sensors)/2)]:
            if ultra_sensor > D_LIM and self.mode == 'forward':# This is under discussion and will be edited later on 
                msg = String()
                msg.data = 'stop'
                self.emergency_pub.publish(msg)
         for ultra_sensor in self.sensors[int(len(self.sensors)/2):]:
            if ultra_sensor > D_LIM and self.mode == 'reverse':# This is under discussion and will be edited later on 
                msg = String()
                msg.data = 'stop'
                self.emergency_pub.publish(msg)               


if __name__ == '__main__':
    ultra = Ultrasonic()
    ultra.start()
