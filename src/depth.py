#!/usr/bin/python2.7

import math
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

face_cascade = cv.CascadeClassifier('/home/zeabus/Desktop/haarcascade_frontalface_default.xml')
img = None
disparity = .1
px,py = 0,0
display = None
baseline = 0
focal_length = 0 
gray = None

def mouse_callback(event, x, y, flags, param):
    global px, py, img
    if event == cv.EVENT_LBUTTONDOWN:
        px, py = x, y
        # if img is not None:
        #     print(px,py)
            
def gray_callback(msg):
    global gray
    bridge = CvBridge()
    tmp = bridge.imgmsg_to_cv2(msg, "mono8")
    tmp = cv.flip(tmp,1)
    gray = tmp.copy()

def callback(msg):
    """
        Convert data from camera to image
    """
    global img, baseline, focal_length, disparity
    bridge = CvBridge()
    tmp = bridge.imgmsg_to_cv2(msg.image)
    tmp = cv.flip(tmp,1)
    tmp = cv.medianBlur(tmp,5)
    kernel = np.ones((5,5),np.int16)
    tmp = cv.erode(tmp,kernel,iterations = 1)
    disparity = tmp.copy()
    tmp = 255*(tmp - tmp.min()) / (tmp.max() - tmp.min())
    tmp = np.uint8(tmp)
    baseline = msg.T
    focal_length = msg.f
    # print(img)
    # print(tmp.max())
    # print(tmp.min())
    
    img = cv.applyColorMap(tmp, cv.COLORMAP_JET)
    



if __name__ == '__main__':
    global img, px, py, baseline, focal_length, disparity, gray
    rospy.init_node('height_estimation', anonymous=False)
    
    topic = "/stereo/disparity"
    topic_gray = "/stereo/left/image_rect"
    rospy.Subscriber(topic, DisparityImage, callback)
    rospy.Subscriber(topic_gray, Image, gray_callback)
    cv.namedWindow('image', flags=cv.WINDOW_NORMAL)
    cv.setMouseCallback('image', mouse_callback)
    font = cv.FONT_HERSHEY_SIMPLEX 
    while not rospy.is_shutdown():
        if img is None:
            continue
        print("Baseline",baseline)
        print("focal length",focal_length)
        faces = face_cascade.detectMultiScale(gray, 1.1, 1)
        r,c = gray.shape
        h_px = 210
        h_mm = 1300
        cv.line(img,(0,h_px),(c-1,h_px),(255,255,255),3)
        print(r,c)
        for (x, y, w, h) in faces:
            cv.rectangle(img, (x, y), (x+w, y+h), (255,255,255), 2)
            cx, cy = x+(w//2), y+(h//2)
            cv.circle(img, (cx,cy), 10, (0,0,0), -1)
            head_depth = baseline * focal_length / disparity[cy,cx]
            base_depth = baseline * focal_length / disparity[h_px,cx]
            # (head_depth - base_depth) =  (cy - h_px)
            h_person = (disparity[cy,cx] - disparity[r - 1,cx]) / disparity[cy,cx] * head_depth
            # h_person  = (head_depth * (disparity[cy,cx] - disparity[h_px,cx]) / disparity[h_px,cx]) + h_mm
            cv.putText(img, "%.2f Meter"%(h_person/1000.), (cx+5,cy+5), font,  
                    1.5, (255,255,255), 3, cv.LINE_AA) 
        cv.imshow("image",img)
        cv.waitKey(1)