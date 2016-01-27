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
from ball_detect_utils import bgr_to_center_radius, plot_center_radius

# setups
left_img = 0
right_img = 0
bridge = CvBridge()
right_ready = False
left_ready = False

# call backs
def left_image_callback(msg):
    global left_img, left_ready
    left_ready = True
    left_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

def right_image_callback(msg):
    global right_img, right_ready
    right_ready = True
    right_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

# subscribers
rospy.Subscriber("/stereo/left/image_raw", Image, left_image_callback)
rospy.Subscriber("/stereo/right/image_raw", Image, right_image_callback)

# publishers
left_ball_visulize_pub = rospy.Publisher('/stereo/left/ball_visulize',
                                         Image, queue_size=10)
right_ball_visulize_pub = rospy.Publisher('/stereo/right/ball_visulize',
                                          Image, queue_size=10)

# init node
rospy.init_node('ball_detect', anonymous=True)

while True:
    # rospy.spinOnce()

    if left_ready:
        left_centers, left_radiuses = bgr_to_center_radius(left_img)
        left_img = plot_center_radius(left_img, left_centers, left_radiuses)
        left_ball_visulize_pub.publish(bridge.cv2_to_imgmsg(left_img, "bgr8"))

    if right_ready:
        right_centers, right_radiuses = bgr_to_center_radius(right_img)
        right_img = plot_center_radius(right_img, right_centers, right_radiuses)
        right_ball_visulize_pub.publish(bridge.cv2_to_imgmsg(right_img, "bgr8"))

    time.sleep(0.1)
