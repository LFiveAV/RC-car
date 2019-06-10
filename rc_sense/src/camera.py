#!/usr/bin/env python

import cv2
import numpy as np

class OnboardCamera(object):
    def __init__(self):
        self.cap = self.open_onboard_camera()

    def open_onboard_camera(self):
        return cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    def get_frame(self):
        if self.cap.isOpened():
            ret_val, frame = self.cap.read();
            return ret_val,frame
        else:
            print ("camera open failed")
            return False,np.array([0])

    def release(self):
        self.cap.release()
