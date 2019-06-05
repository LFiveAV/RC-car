#!/usr/bin/env python

import rospy
import numpy as np
from custom_messages.msg import Image



class CvBridge(object):
    def cv2_to_imgmsg(self,frame):
        imgmsg = Image()
        msg = [-1]
        for i in frame.shape:
            msg.append(i)
        msg[0] = len(msg)-1
        size = np.prod(msg[1:])
        frame_reshaped = frame.reshape((1,size))
        msg += frame_reshaped[0].tolist()
        imgmsg.data = msg
        return imgmsg


    def imgmsg_to_cv2(self,msg):
        size = msg.pop(0)
        frame_dim = []
        for __ in range(size):
            frame_dim.append(msg.pop(0))
        frame = np.asarray(msg)
        frame = frame.reshape(frame_dim)
        return frame
