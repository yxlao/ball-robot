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
from cv_bridge import CvBridge, CvBridgeError
import time
#from ball_detect_utils import hsv_to_center_radius, plot_center_radius
from detect_utils import orange_hsv_lows, orange_hsv_highs, green_hsv_lows, green_hsv_highs, hsv_to_ball_center_radius, plot_center_radius, plot_targets
from detect_utils import hsv_to_targets
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

# setups
left_img = 0
right_img = 0
bridge = CvBridge()
right_ready = False
left_ready = False

last_x = 0
last_y = 0
last_z = 0

last_command = "s"

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
disparity_pub = rospy.Publisher('/stereo/my_disparity', Image, queue_size=10)
ball_coord_pub = rospy.Publisher('/ball_coords', Vector3, queue_size=10)
bucket_coord_pub = rospy.Publisher('/bucket_coords', Vector3, queue_size=5)
state_cmd_pub = rospy.Publisher('/state_cmds', String, queue_size=10)
ball_in_sight_pub = rospy.Publisher('/is_ball_in_sight', String, queue_size=10)
bucket_in_sight_pub = rospy.Publisher(
    '/is_bucket_in_sight', String, queue_size=10)
lower_cam_ball_in_sight_pub = rospy.Publisher(
    'lower_cam_is_ball_in_sight', String, queue_size=10)
lower_cam_ball_coord_pub = rospy.Publisher(
    'lower_cam_ball_coords', Vector3, queue_size=10)
tf_broadcaster = tf.TransformBroadcaster()

# init node
rospy.init_node('ball_detect', anonymous=True)


def get_disparity_image(left_img, right_img):
    # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET,
                          ndisparities=16, SADWindowSize=21)
    disparity_img = stereo.compute(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY),
                                   cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY))
    disparity_img = np.abs(disparity_img)
    disparity_img = disparity_img.astype(np.uint8)

    return disparity_img
ball_coords = Vector3()
bucket_coords = Vector3()
while not rospy.is_shutdown():
    # rospy.spinOnce()
    if not (left_ready and right_ready):
        time.sleep(0.1)
        continue
    if left_ready:
        im_hsv = cv2.cvtColor(left_img, cv2.COLOR_BGR2HSV)
        # get centers and radiuses
        # green_centers, green_radiuses = hsv_to_ball_center_radius(im_hsv,
        # hsv_lows=green_hsv_lows,
        # hsv_highs=green_hsv_highs)
        # plot center and radius
        # im_bgr = plot_center_radius(left_img, green_centers, green_radiuses,
        # color="green")

        # get centers and radiuses
        # orange_centers, orange_radiuses = hsv_to_ball_center_radius(im_hsv,
        # hsv_lows=orange_hsv_lows,
        # hsv_highs=orange_hsv_highs)
        # plot center and radius
        # im_bgr = plot_center_radius(im_bgr, orange_centers, orange_radiuses,
        # color="orange")

        # draw targests
        targets = hsv_to_targets(im_hsv)
        im_bgr = plot_targets(left_img, targets)
        # left_centers, left_radiuses = hsv_to_center_radius(
        # im_hsv, orange_hsv_lows, orange_hsv_highs)
        # left_img = plot_center_radius(
        # left_img, left_centers, left_radiuses, color="orange")
        left_ball_visulize_pub.publish(bridge.cv2_to_imgmsg(im_bgr, "bgr8"))
        #left_green_centers, left_green_rediuses = hsv_to_center_radius(left_img, green_hsv_lows, green_hsv_highs)

    if right_ready:
        im_hsv_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2HSV)
        # right_centers, right_radiuses = hsv_to_center_radius(
        #    im_hsv_right, orange_hsv_lows, orange_hsv_highs)
        #right_img = plot_center_radius(right_img, right_centers, right_radiuses)
        # get centers and radiuses
        # green_centers, green_radiuses = hsv_to_ball_center_radius(im_hsv_right,
        # hsv_lows=green_hsv_lows,
        # hsv_highs=green_hsv_highs)
        # plot center and radius
        # im_bgr = plot_center_radius(right_img, green_centers, green_radiuses,
        # color="green")

        # get centers and radiuses
        # orange_centers, orange_radiuses = hsv_to_ball_center_radius(im_hsv_right,
        # hsv_lows=orange_hsv_lows,
        # hsv_highs=orange_hsv_highs)
        # plot center and radius
        # im_bgr = plot_center_radius(im_bgr, orange_centers, orange_radiuses,
        # color="orange")

        # draw targests
        right_targets = hsv_to_targets(im_hsv_right)
        im_bgr = plot_targets(right_img, right_targets)
        right_ball_visulize_pub.publish(bridge.cv2_to_imgmsg(im_bgr, "bgr8"))

    # disparity_img = get_disparity_image(left_img, right_img)
    # print (disparity_img)
    # disparity_pub.publish(bridge.cv2_to_imgmsg(disparity_img, "mono8"))

    global last_command
    if targets['green'] != None or targets['orange'] != None:
        if targets['orange'] != None and targets['orange']['d'] < 80:
            ball_coords.x = float(targets['orange']['x'] - 160.0) / 100
            ball_coords.z = float(targets['orange']['y'] - 120.0) / 100
            ball_coords.y = targets['orange']['d']
        elif targets['green'] != None and targets['green']['d'] < 80:
            ball_coords.x = float(targets['green']['x'] - 160.0) / 100
            ball_coords.z = float(targets['green']['y'] - 120.0) / 100
            ball_coords.y = targets['green']['d']

        # broadcast location
        tf_broadcaster.sendTransform((ball_coords.x, ball_coords.y, ball_coords.z),
                                     tf.transformations.quaternion_from_euler(
                                         0, 0, 0),
                                     rospy.Time.now(), "ball_lower_cam", "camera_frame")
        lower_cam_ball_in_sight_pub.publish("true")
        lower_cam_ball_coord_pub.publish(ball_coords)
    else:
        lower_cam_ball_in_sight_pub.publish("false")
    if right_targets['green'] != None or right_targets['orange'] != None:
        if right_targets['orange'] != None and right_targets['orange']['d'] < 100:
            ball_coords.x = float(right_targets['orange']['x'] - 160.0) / 100
            ball_coords.z = float(right_targets['orange']['y'] - 120.0) / 100
            ball_coords.y = right_targets['orange']['d']
        elif right_targets['green'] != None and right_targets['green']['d'] < 100:
            ball_coords.x = float(right_targets['green']['x'] - 160.0) / 100
            ball_coords.z = float(right_targets['green']['y'] - 120.0) / 100
            ball_coords.y = right_targets['green']['d']

        # broadcast location
        tf_broadcaster.sendTransform((ball_coords.x, ball_coords.y, ball_coords.z),
                                     tf.transformations.quaternion_from_euler(
                                         0, 0, 0),
                                     rospy.Time.now(), "ball_upper_cam", "camera_frame")

        #print(x, y, z)
        ball_in_sight_pub.publish("true")
        ball_coord_pub.publish(ball_coords)

    else:
        ball_in_sight_pub.publish("false")
    if right_targets['bucket'] != None:
        bucket_coords.x = float(right_targets['bucket']['x'] - 160.0) / 100
        bucket_coords.z = float(right_targets['bucket']['y'] - 120.0) / 100
        bucket_coords.y = right_targets['bucket']['d']
        bucket_coord_pub.publish(bucket_coords)
        bucket_in_sight_pub.publish("true")
    else:
        bucket_in_sight_pub.publish("false")

    time.sleep(0.1)
