#!/usr/bin/env python
# license removed for brevity
import numpy as np
import cv2
import rospy
import serial
import cv2
import cv2.cv as cv
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# cap = cv2.VideoCapture(1) #left
# cap2 = cv2.VideoCapture(0) #right
rospy.init_node('depth_node', anonymous=True)
leftImg = 0
rightImg = 0
bridge = CvBridge()
rightReady = False
leftReady = False
print "start"


def left_image_callback(msg):
    global leftImg, leftReady
    # print "left"
    leftReady = True
    leftImg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # ROS_ERROR("LEFT ")


def right_image_callback(msg):
    global rightImg, rightReady
    # print "right"
    rightReady = True
    rightImg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # ROS_ERROR("RIGHT")

h = np.zeros((160, 120, 1), np.uint8)
s = np.zeros((160, 120, 1), np.uint8)
v = np.zeros((160, 120, 1), np.uint8)
rospy.Subscriber("stereo/left/image_rect_color", Image, left_image_callback)
rospy.Subscriber("stereo/right/image_rect_color", Image, right_image_callback)
print "Subscriber started"

while(True):
    if (not leftReady) or (not rightReady):
        # rospy.spin()
        continue
    # Capture frame-by-frame
    # ret, frame0 = cap.read()
    # ret2, frame02 = cap2.read()
    # Resize the resolution of image
    frame = cv2.resize(leftImg, (160, 120), interpolation=cv2.INTER_CUBIC)
    frame2 = cv2.resize(rightImg, (160, 120), interpolation=cv2.INTER_CUBIC)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
    h2, s2, v2 = cv2.split(hsv2)

    # find orange
    for y in range(1, 160):
        for x in range(1, 120):
            if h[x, y] > 3 and h[x, y] < 8 and s[x, y] > 156 and s[x, y] < 188 and v[x, y] > 144 and v[x, y] < 214:
                h[x, y] = h[x, y]
            else:
                h[x, y] = 255
            if h2[x, y] > 3 and h2[x, y] < 8 and s2[x, y] > 156 and s2[x, y] < 188 and v2[x, y] > 144 and v2[x, y] < 214:
                h2[x, y] = h2[x, y]
            else:
                h2[x, y] = 255
    ret, thresh = cv2.threshold(h, 60, 255, 0)
    ret2, thresh2 = cv2.threshold(h2, 60, 255, 0)
    # erode
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
    eroded = cv2.erode(thresh, kernel)
    eroded2 = cv2.erode(thresh2, kernel)
    # dilate
    dilated = cv2.dilate(eroded, kernel)
    dilated2 = cv2.dilate(eroded2, kernel)
    # find contour
    image, contours = cv2.findContours(dilated, 1, 2)
    image2, contours2 = cv2.findContours(dilated2, 1, 2)

    cnt = contours[0]
    cnt2 = contours2[0]
    print cnt

    # Calculte the center of the ball
    (center_x, center_y), radius = cv2.minEnclosingCircle(cnt)
    (center_x2, center_y2), radius2 = cv2.minEnclosingCircle(cnt2)

    center_x = center_x * 4
    center_y = center_y * 4
    radius = radius * 4
    center_x2 = center_x2 * 4
    center_y2 = center_y2 * 4
    radius2 = radius2 * 4

    center = (int(center_x), int(center_y))
    center2 = (int(center_x2), int(center_y2))
    radius = int(radius)
    radius2 = int(radius2)
    # Plot the position of the ball
    if radius > 3 and radius2 > 3:
        cv2.circle(leftImg, center, radius, (0, 255, 0), 2)
        cv2.circle(rightImg, center2, radius2, (0, 255, 0), 2)
        # calculate depth
        f = 10
        T = 8
        k = 1
        if center_y2 != center_y:
            depth = f * T * k / abs(center_y2 - center_y)
        if depth > 0:
            print depth
    #   print center_x
    #  print center_y
    #   print center_x2
    #  print center_y2
    #gray = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('left', leftImg)
    cv2.imshow('right', rightImg)
    key = cv2.waitKey(10)
    if key == 27:
        break

# When everything done, release the capture
# cap.release()
# cap2.release()
cv2.destroyAllWindows()
