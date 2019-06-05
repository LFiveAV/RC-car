#!/usr/bin/env python

import rospy
import time
import cv2
from cv_bridge import CvBridge
from camera import OnboardCamera
from custom_messages.msg import Image



class ImagePublisher(object):
    def __init__(self):
        self.camera = OnboardCamera()
        self.pub = rospy.Publisher("image_topic",Image)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)

    def start(self):
        while not rospy.is_shutdown():
            ret_val, frame = self.camera.get_frame()
            if ret_val is True:
                image = self.edit_image(frame)
                self.publish_image(image)
            self.rate.sleep()

    def publish_image(self,image):
        self.pub.publish(self.bridge.cv2_to_imgmsg(image))

    def edit_image(self,frame):
        # Edit image here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return gray



if __name__ == '__main__':
    rospy.init_node('image_publsiher', anonymous=True)
    ip = ImagePublisher()
    ip.start()
    ip.camera.release()
