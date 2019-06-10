#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from camera import OnboardCamera
from sensor_msgs.msg import Image



class ImagePublisher(object):
    def __init__(self):
        self.camera = OnboardCamera()
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("image_topic",Image,queue_size=2)
        self.rate = rospy.Rate(5)

    def start(self):
        while not rospy.is_shutdown():
            ret_val, frame = self.camera.get_frame()
            if ret_val is True:
                #image = self.edit_image(frame)
                self.publish_image(frame)
            self.rate.sleep()

    def publish_image(self,image):
        # Encodings
        # Grayscale: MONO8
        # Color: RGB8
        encoding = 'MONO8'
        self.pub.publish(self.bridge.cv2_to_imgmsg(image,encoding))

    def edit_image(self,frame):
        # Edit image here
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return gray
        except cv2.error as e:
            print 'edit image failed'
            print e
            return frame



if __name__ == '__main__':
    rospy.init_node('image_publisher_node', anonymous=True)
    ip = ImagePublisher()
    ip.start()
    ip.camera.release()
