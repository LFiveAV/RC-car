#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image


VALID_ENCODINGS = ['RGB8','MONO8','MONO16']

class CvBridge(object):
    def cv2_to_imgmsg(self,frame,encoding):
        if encoding in VALID_ENCODINGS:
            msg = Image()
            channels = num_channels(encoding)
            height = frame.shape[0]
            width = frame.shape[1]
            size = height*width*channels
            frame_reshaped = frame.reshape((1,size))
            msg.height = height
            msg.width = width
            msg.data = frame_reshaped[0].tolist()
            msg.encoding = encoding
            return msg
        else:
            print 'Unknown encoding: {}'.format(encoding)
            print 'Valid encodings are ' + str(VALID_ENCODINGS)



    def imgmsg_to_cv2(self,msg):
        height = msg.height
        width = msg.width
        channels = num_channels(msg.encoding)
        frame_dim = (height,width,channels)
        frame = np.asarray(msg.data)
        frame = frame.reshape((frame_dim))
        return frame


def num_channels(encoding):
    if encoding == 'MONO8' or encoding == 'MONO16':
        return 1
    if encoding == 'RGB8':
        return 3
