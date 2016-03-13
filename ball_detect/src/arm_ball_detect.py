#!/usr/bin/env python
# license removed for brevity
import serial
import cv2
import cv2.cv as cv
import sys
import numpy as np
import tf
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import time
from detect_utils import orange_hsv_lows, orange_hsv_highs, green_hsv_lows, green_hsv_highs
from detect_utils import hsv_to_ball_center_radius, plot_center_radius, plot_targets
from detect_utils import hsv_to_targets
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

# setups
im_bgr = 0
im_ready = False
bridge = CvBridge()

def image_callback(msg):
    global im_bgr, im_ready
    im_ready = True
    im_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

# subscribers
rospy.Subscriber("/arm_cam/image_raw/", Image, image_callback)

# publishers
arm_ball_visulize_pub = rospy.Publisher('/arm_cam/ball_visulize',
                                        Image, queue_size=10)
arm_has_ball_pub = rospy.Publisher('/arm_cam/has_ball', Bool, queue_size=10)

# init node
rospy.init_node('arm_ball_detect', anonymous=True)

while not rospy.is_shutdown():
    # rospy.spinOnce()
    if not (im_ready):
        time.sleep(0.1)
        continue

    if im_ready:
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
        arm_targets = hsv_to_targets(im_hsv)
        im_bgr = plot_targets(im_bgr, arm_targets)
        arm_ball_visulize_pub.publish(bridge.cv2_to_imgmsg(im_bgr, "bgr8"))
        arm_has_ball_pub.publish(True)

    time.sleep(0.1)
