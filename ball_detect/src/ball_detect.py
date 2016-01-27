#!/usr/bin/env python
# license removed for brevity
import serial
import cv2
import cv2.cv as cv
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

pub = rospy.Publisher('ball_left', Image, queue_size=10)
rospy.init_node('ball_detect_node', anonymous=True)

leftImg = 0
rightImg = 0
bridge = CvBridge()
rightReady = False
leftReady = False


def left_image_callback(msg):
    global leftImg, leftReady
    leftReady = True
    leftImg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


def right_image_callback(msg):
    global rightImg, rightReady
    rightReady = True
    rightImg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

rospy.Subscriber("/stereo/left/image_raw", Image, left_image_callback)
# rospy.Subscriber("right_cam/image_raw", Image, right_image_callback)


while True:
    # rospy.spinOnce()
    # if (not leftReady) or (not rightReady):
    #     continue
    if (not leftReady):
        continue
    # disparity = getDisparity(leftImg, rightImg, "BM")

    # gray_image = cv2.cvtColor(leftImg, cv2.COLOR_BGR2GRAY)
    disparity_msg = bridge.cv2_to_imgmsg(leftImg, "rgb8")

    pub.publish(disparity_msg)
    time.sleep(0.1)