#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera(object):
    def __init__(self):
        self.cap = self.open_onboard_camera()

    def open_onboard_camera(self):
        return cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

    def get_frame(self):
        if self.cap.isOpened():
            ret_val, frame = self.cap.read();
            return ret_val, frame
        else:
            print ("camera open failed")
            return False, 0
    
    def release(self):
        self.cap.release()

class ImagePublisher(object):
    def __init__(self):
        self.camera = Camera()
        self.pub = rospy.Publisher("image_topic",Image)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)

    def start(self):
        while not rospy.is_shutdown():
            ret_val, frame = self.camera.get_frame()
            if ret_val is True:
                image = self.edit_image(frame)
                self.publish_image(self,image)
            self.rate.sleep()

    def publish_image(self,image):
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(image,"passthrough"))
        except CvBridgeError as e:
            print (e)

    def edit_image(self,frame):
        # Edit image here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return gray
            


if __name__ == '__main__':
    rospy.init_node('image_publsiher', anonymous=True)
    ip = ImagePublisher()
    ip.start()
    ip.camera.release()
