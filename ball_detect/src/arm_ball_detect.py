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
#from detect_utils import orange_hsv_lows, orange_hsv_highs, green_hsv_lows, green_hsv_highs
from detect_utils import hsv_to_ball_center_radius, plot_center_radius, plot_targets
from detect_utils import arm_hsv_to_targets
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

#green_hsv_lows = (46, 104, 175)
#green_hsv_highs = (52, 157, 255)
#green_hsv_lows = (33, 108, 85)
#green_hsv_highs = (54, 171, 188)
#green_hsv_lows = (48, 126, 46)
#green_hsv_highs = (59, 188, 204)
#3f
green_hsv_lows = (32, 6, 179)
green_hsv_highs = (43, 163, 255)



#orange_hsv_lows = (7, 96, 161)
#orange_hsv_highs = (16, 183, 255)

#orange_hsv_lows = (6, 147, 117)
#orange_hsv_highs = (10, 182, 239)
#3rd floor
orange_hsv_lows = (8, 10, 171)
orange_hsv_highs = (37, 181, 255)

# mar 15 value
hsv_lows = (3, 78, 110)
hsv_highs = (18, 188, 255)


min_dist = 5.
max_dist = 15.

min_center = 0.35
max_center = 0.65

while not rospy.is_shutdown():
    # rospy.spinOnce()
    if not (im_ready):
        time.sleep(0.1)
        continue

    if im_ready:
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
        im_height = float(im_hsv.shape[0])
        im_width = float(im_hsv.shape[1])
        targets = arm_hsv_to_targets(im_hsv,
                                    green_hsv_lows=green_hsv_lows,
                                    green_hsv_highs=green_hsv_highs,
                                    orange_hsv_lows=orange_hsv_lows,
                                    orange_hsv_highs=orange_hsv_highs,
                                    bucket_hsv_lows=(0, 0, 0),
                                    bucket_hsv_highs=(0, 0, 0))

        # tell if a ball inside
        has_ball = False
        if targets['orange'] and targets['orange']['d']:
            if (targets['orange']['d'] > min_dist and
                targets['orange']['d'] < max_dist and
                targets['orange']['x'] / im_width > min_center and
                targets['orange']['x'] / im_width < max_center):
                has_ball = True
        if targets['green'] and targets['green']['d']:
            if (targets['green']['d'] > min_dist and
                targets['green']['d'] < max_dist and
                targets['green']['x'] / im_width > min_center and
                targets['green']['x'] / im_width < max_center):
                has_ball = True

        im_bgr = plot_targets(im_bgr, targets)
        arm_ball_visulize_pub.publish(bridge.cv2_to_imgmsg(im_bgr, "bgr8"))
        arm_has_ball_pub.publish(has_ball)

    time.sleep(0.1)
