#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Last Update: Wed, 15 August 2018

@author: I AVL
"""


import rospy
import logging
import sys
import time
import math
import argparse
import cv2
import numpy as np

from std_msgs.msg import String

"""
from Adafruit_BNO055 import BNO055
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8
"""

pub_rate = 10 

def parse_cli_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video_device", dest="video_device",
                        help="Video device # of USB webcam (/dev/video?) [0]",
                        default=0, type=int)
    arguments = parser.parse_args()
    return arguments


# Use the Jetson onboard camera
def open_onboard_camera():
    return cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")


# Open an external usb camera /dev/videoX

def open_camera_device(device_number):
    return cv2.VideoCapture(device_number)








def publish_cam_data(video_capture):
    # cam_msg = cam()
    pub = rospy.Publisher('cam_data', String, queue_size=5)       #Queue size??
    #rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(pub_rate) # 10hz
    
    while not rospy.is_shutdown():
        
     if video_capture.isOpened():
        windowName = "CannyDemo"
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName,2500,1000)
        cv2.moveWindow(windowName,0,0)
        cv2.setWindowTitle(windowName,"Jetson Watches")
        showWindow=3  # Show all stages
        showHelp = True
        font = cv2.FONT_HERSHEY_PLAIN
        helpText=" 'i' Camera View, 'c' Canny Detection, 'a' All Stages, '+' +edgeThreshold, '-' -Threshold,'4' Hide Help, 'Esc' to Quit"
        edgeThreshold=100
        showFullScreen = False
        while True:
            if cv2.getWindowProperty(windowName, 0) < 0: # Check to see if the user closed the window
                # This will fail if the user closed the window; Nasties get printed to the console
                break;
            ret_val, frame = video_capture.read();
            hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur=cv2.GaussianBlur(hsv,(7,7),1.5)
            edges=cv2.Canny(blur,0,edgeThreshold)
            if showWindow == 3:  # Need to show the 3 stages
                # Composite the 2x2 window
                # Feed from the camera is RGB, the others gray
                # To composite, convert gray images to color. 
                # All images must be of the same type to display in a window
                frameRs=cv2.resize(frame, (640,640))
                hsvRs=cv2.resize(hsv,(640,640))
                vidBuf = np.concatenate((frameRs, cv2.cvtColor(hsvRs,cv2.COLOR_GRAY2BGR)), axis=1)
                edgesRs=cv2.resize(edges,(640,640))
                vidBuf = np.concatenate( (vidBuf, cv2.cvtColor(edgesRs,cv2.COLOR_GRAY2BGR)), axis=1)

            if showWindow==1: # Show Camera Frame
                displayBuf = frame 
            elif showWindow == 2: # Show Canny Edge Detection
                displayBuf = edges
            elif showWindow == 3: # Show All Stages
                displayBuf = vidBuf

            if showHelp == True:
                cv2.putText(displayBuf, helpText, (11,20), font, 1.0, (32,32,32), 4, cv2.LINE_AA)
                cv2.putText(displayBuf, helpText, (10,20), font, 1.0, (240,240,240), 1, cv2.LINE_AA)
            cv2.imshow(windowName,displayBuf)
            key=cv2.waitKey(10)
            
            if key==105: # i key, show frame
                cv2.setWindowTitle(windowName,"Camera Feed")
                showWindow=1
            elif key==99: # c key, show Canny
                cv2.setWindowTitle(windowName,"Canny Edge Detection")
                showWindow=2
            elif key==97: # a key, show Stages
                cv2.setWindowTitle(windowName,"Camera, Gray scale, Canny Edge Detection")
                showWindow=3
            elif key==52: # 4 key, toggle help
                showHelp = not showHelp
            elif key==45: # , lower canny edge threshold
                edgeThreshold=max(0,edgeThreshold-1)
                print ('Canny Edge Threshold Maximum: ',edgeThreshold)
            elif key==43: # , raise canny edge threshold
                edgeThreshold=edgeThreshold+1
                print ('Canny Edge Threshold Maximum: ', edgeThreshold)
            elif key==74: # Toggle fullscreen
                if showFullScreen == False : 
                    cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                else:
                    cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL) 
                showFullScreen = not showFullScreen
            elif key == 27: # Check for ESC key
                cv2.destroyAllWindows()
                break ;
    else:
     print ("camera open failed")




if __name__ == '__main__':
    rospy.init_node('cam_node', anonymous=True)
    arguments = parse_cli_args()
    print("Called with args:")
    print(arguments)
    print("OpenCV version: {}".format(cv2.__version__))
    print("Device Number:",arguments.video_device)
    if arguments.video_device==0:
      video_capture=open_onboard_camera()
    else:
      video_capture=open_camera_device(arguments.video_device)
    video_capture.release()
    cv2.destroyAllWindows()

try:
	publish_cam_data(video_capture)
	print("video capture?")
except rospy.ROSInterruptException:
	pass 


