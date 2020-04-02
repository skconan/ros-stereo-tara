#!/usr/bin/python2.7

import math
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
img = None

def callback(msg):
    """
        Convert data from camera to image
    """
    global img
    print(msg)

if __name__ == '__main__':
    rospy.init_node('height_estimation', anonymous=False)
    
    topic = "/stereo/disparity"
    rospy.Subscriber(topic, DisparityImage, callback)
    rospy.spin()