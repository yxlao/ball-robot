#!/usr/bin/env python
# license removed for brevity
import serial
import cv2
import cv2.cv as cv
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image

pub = rospy.Publisher('disparity', Float32, queue_size=10)
rospy.init_node('disparity_node', anonymous=True)
leftImg = 0
rightImg = 0


def left_image_callback(msg):
    global leftImg
    leftImg = msg.data

def right_image_callback(msg):
    global rightImg
    rightImg = msg.data


def getDisparity(imgLeft, imgRight, method="BM"):

    gray_left = cv2.cvtColor(imgLeft, cv.CV_BGR2GRAY)
    gray_right = cv2.cvtColor(imgRight, cv.CV_BGR2GRAY)
    print gray_left.shape
    c, r = gray_left.shape
    if method == "BM":
        sbm = cv.CreateStereoBMState()
        disparity = cv.CreateMat(c, r, cv.CV_32F)
        sbm.SADWindowSize = 9
        sbm.preFilterType = 1
        sbm.preFilterSize = 5
        sbm.preFilterCap = 61
        sbm.minDisparity = -39
        sbm.numberOfDisparities = 112
        sbm.textureThreshold = 507
        sbm.uniquenessRatio= 0
        sbm.speckleRange = 8
        sbm.speckleWindowSize = 0

        gray_left = cv.fromarray(gray_left)
        gray_right = cv.fromarray(gray_right)

        cv.FindStereoCorrespondenceBM(gray_left, gray_right, disparity, sbm)
        disparity_visual = cv.CreateMat(c, r, cv.CV_8U)
        cv.Normalize(disparity, disparity_visual, 0, 255, cv.CV_MINMAX)
        disparity_visual = np.array(disparity_visual)

    elif method == "SGBM":
        sbm = cv2.StereoSGBM()
        sbm.SADWindowSize = 9;
        sbm.numberOfDisparities = 96;
        sbm.preFilterCap = 63;
        sbm.minDisparity = -21;
        sbm.uniquenessRatio = 7;
        sbm.speckleWindowSize = 0;
        sbm.speckleRange = 8;
        sbm.disp12MaxDiff = 1;
        sbm.fullDP = False;

        disparity = sbm.compute(gray_left, gray_right)
        disparity_visual = cv2.normalize(disparity, alpha=0, beta=255, norm_type=cv2.cv.CV_MINMAX, dtype=cv2.cv.CV_8U)

    return disparity_visual
disparity_msg = Image()
while True:
    if leftImg == 0 or rightImg == 0:
        continue
    disparity_msg.data = getDisparity(leftImg, rightImg, "BM")
    pub.publish(disparity_msg)

# imgLeft = cv2.imread(sys.argv[1])
# imgRight = cv2.imread(sys.argv[2])
# try:
#     method = sys.argv[3]
# except IndexError:
#     method = "BM"

# disparity = getDisparity(imgLeft, imgRight, method)
# cv2.imshow("disparity", disparity)
# cv2.waitKey(0)