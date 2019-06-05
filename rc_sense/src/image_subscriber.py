#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSub(object):
    def __init__(self):
        self.bridge = Bridge()

    def start():
        self.image_listener(self):

    def image_callback(self,data):
        try:
            image = self.bridge.imgmsg_cv2(data,"passthrough")
        except CvBridgeError as e:
            print (e)

        self.show_image(image)

    def show_image(self,image):
        cv2.imshow("frame",image)
        cv2.waitKey(3)
            

    def image_listener(self):
        rospy.Subscriber("image_topic",Image,self.image_callback)
        
if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    imgsub = ImageSub()
    imgsub.start()
