#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed May 23 09:35:10 2018

@author: nvidia
"""

import rospy
import curses

#from std_msgs.msg import Float32
#from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray as TwoFloat32


class RemoteController(object):
    def __init__(self, control_mode="pwm mode"):
        self.emergency_pub = rospy.Publisher('emergency_stop', String, queue_size=3)
        self.mode_pub = rospy.Publisher('mode',String,queue_size=2)
        self.control_mode = control_mode
        self.self_drive_key = 'a'
        self.remote_drive_key = 'm'

        if self.control_mode == "pwm mode":
            self.remote_pub = rospy.Publisher('remote', TwoFloat32, queue_size=2)
            self.speed_inc = 2
            self.angle_inc = 2
            self.speed = 0
            self.steering = 0


        elif self.control_mode == "metric mode":
            self.pub_steering = rospy.Publisher('steering_command', Float32, queue_size=2)
            self.pub_speed = rospy.Publisher('speed_command', Float32, queue_size=2)
            self.speed_inc = 0.20
            self.angle_inc = 2
            self.speed = 0
            self.steering = 0

    def emergency_stop(self):
        self.emergency_pub.publish("stop")

    def remote_control(self):

        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()                 # respond to keys immediately
        screen.keypad(True)             # map arrow keys to special values

        while not rospy.is_shutdown():
            drive_command = screen.getch()
            msg = TwoFloat32()
            msg = [0,0]

            if self.control_mode == "pwm mode":
                if drive_command == 32:    # for emergency stop: hit space
                    self.emergency_stop()
                    print("stop!")
                elif drive_command == curses.KEY_UP:
                    msg[0] = self.speed_inc
                elif drive_command == curses.KEY_DOWN:
                    msg[0] = -self.speed_inc
                elif drive_command == curses.KEY_LEFT:
                    msg[1] = self.angle_inc
                elif drive_command == curses.KEY_RIGHT:
                    msg[1] = -self.angle_inc

            elif self.control_mode == "metric mode":
                if drive_command == 32:    # for emergency stop: hit space
                    self.speed = 0
                    self.emergency_stop()
                    print("stop!")
                elif drive_command == curses.KEY_UP:
                    self.speed = self.speed + self.speed_inc
                    self.pub_speed.publish(self.speed)
                elif drive_command == curses.KEY_DOWN:
                    self.speed = self.speed - self.speed_inc
                    self.pub_speed.publish(self.speed)
                elif drive_command == curses.KEY_LEFT:
                    self.steering = self.steering + self.angle_inc
                    self.pub_steering.publish(self.steering)
                elif drive_command == curses.KEY_RIGHT:
                    self.steering = self.steering - self.angle_inc
                    self.pub_steering.publish(self.steering)

            self.remote_pub(msg)

            if drive_command == ord(self.self_drive_key):
                self.mode_pub('self_drive')
            elif drive_command == ord(self.remote_drive_key):
                self.mode_pub('remote')

        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        self.emergency_stop()


if __name__ == '__main__':
    rospy.init_node('remote_controller_node')

    print("[Node] remote_controller started")

    controller = RemoteController("pwm mode")
    controller.remote_control()
