#!/usr/bin/env python
# license removed for brevity
"""
To run this:
    $ python ball_detect_utils.py
"""
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
from sensor_msgs.msg import Image

rospy.init_node('depth_node', anonymous=True)
hsv_lows_default = (0, 146, 120)
hsv_highs_default = (16, 255, 255)
rightImg = 0
leftImg = 0
bridge = CvBridge()
rightReady = False
leftReady = False


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
rospy.Subscriber("stereo/right/image_rect_color", Image, right_image_callback)
rospy.Subscriber("stereo/left/image_rect_color", Image, left_image_callback)


def calculate_depth(center1, center2, radius, radius2):
    center_x = center1[0]
    center_y = center1[1]
    center_x2 = center2[0]
    center_y2 = center2[1]

    f = 10
    T = 8
    k = 1000
    if center_y2 != center_y:
        depth = f * T * k / (abs(center_y2 - center_y) * radius)
        if True:
            # print depth
            # print x and z
            kx = 50
            ky = 0.0005
            x = (center_x - center_y) * kx / depth
            z = (240 - center_y) * ky * depth
            # print x
            # print z

    # center = (int(center_x),int(center_y))
    # center2 = (int(center_x2),int(center_y2))
    # radius = int(radius)
    # radius2 = int(radius2)
    # Plot the position of the ball
    # if radius>1 and radius2>1:
    # cv2.circle(leftImg,center,radius,(0,255,0),2)
    # cv2.circle(rightImg,center2,radius2,(0,255,0),2)
    # calculate depth
    #     f=10
    #     T=8
    #     k=10000
    #     if center_y2 != center_y:
    #         depth=f*T*k/abs(center_y2-center_y)
    #     if True:
            # print depth
    br = tf.TransformBroadcaster()
    br.sendTransform((1.0 * x / 20, 1.0 * depth / 20, 1.0 * z / 20),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "ball",
                     "base1")
    return (x, depth, z)


def bgr_to_center_radius(im_bgr,
                         hsv_lows=hsv_lows_default,
                         hsv_highs=hsv_highs_default,
                         surpress_when_large=True):
    """
    Detect ball of from bgr image, returns centers and radius for circles
    """
    # resize
    # im_bgr = cv2.resize(im_bgr, (640, 480), interpolation=cv2.INTER_CUBIC)

    # convert to hsv
    im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

    # mask by threshold
    im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
    im_mask = cv2.medianBlur(im_mask, 5)
    # erode
    im_mask = cv2.erode(im_mask, None, iterations=2)
    # dilate
    im_mask = cv2.dilate(im_mask, None, iterations=2)

    # find contours
    contours = cv2.findContours(im_mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

    # get centers and radiuses
    centers = []
    radiuses = []

    for countour in contours:
        center, radius = cv2.minEnclosingCircle(countour)
        centers.append((int(center[0]), int(center[1])))  # a tuple
        radiuses.append(int(radius))  # an int

    if surpress_when_large and len(radiuses) > 0:
        max_radius = max(radiuses)
        if max_radius * 2 > im_hsv.shape[0] * 0.4:
            centers_new = []
            radiuses_new = []
            for center, radius in zip(centers, radiuses):
                if radius * 2 > im_hsv.shape[0] * 0.05:
                    centers_new.append(center)
                    radiuses_new.append(radius)
            centers = centers_new
            radiuses = radiuses_new

    return (centers, radiuses)


def plot_center_radius(im, centers, radiuses):
    """
    Plot circles of centers and radius to im
    """
    # plot center and radius
    for center, radius in zip(centers, radiuses):
        if radius > 2:
            cv2.circle(im, center, radius, (0, 255, 0), 2)
    return im


def plot_center_radius2(im2, centers2, radiuses2):
    """
    Plot circles of centers and radius to im
    """
    # plot center and radius
    for center2, radius2 in zip(centers2, radiuses2):
        if radius2 > 2:
            cv2.circle(im2, center2, radius2, (0, 255, 0), 2)
    return im2


if __name__ == '__main__':
    # set camera
    # camera = cv2.VideoCapture(0)

    # main loop
    while(True):
        if (not rightReady) or (not leftReady):
            # rospy.spin()
            continue
        # read frame
        # (_, im_bgr) = camera.read()
        # print type(rightImg)
        try:
            im_bgr = rightImg  # cv2.cv.QueryFrame(rightImg)
            im_bgr = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2RGB)
            im_bgr_left = cv2.cvtColor(leftImg, cv2.COLOR_BGR2RGB)
            # get centers and radiuses
            centers, radiuses = bgr_to_center_radius(im_bgr)
            centers_left, radiuses_left = bgr_to_center_radius(im_bgr_left)

            # plot center and radius
            im_bgr = plot_center_radius(im_bgr, centers, radiuses)
            im_bgr_left = plot_center_radius2(im_bgr_left, centers_left, radiuses_left)
            # print out the depth
            # try:
            if len(centers) > 0 and len(centers_left) > 0:
                print centers[0]
                print centers_left[0]
                calculate_depth(
                    centers[0], centers_left[0], radiuses[0], radiuses_left[0])
            # except IndexError:
                # a=1
            # print centers
            # print centers_left
            # display the resulting frame
            cv2.imshow('frame', im_bgr)
            cv2.imshow('frame2', im_bgr_left)
            # print centers
            key = cv2.waitKey(10)
            if key == 27:
                break
        except KeyboardInterrupt:
            break

    # when everything done, release the camera
    # camera.release()
    cv2.destroyAllWindows()
