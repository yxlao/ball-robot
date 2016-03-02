"""
Mouse coordinate:
x=0,y=0 -----------> x=640,y=0
|
V
x=0,y=480 ---------- x=640,y=480

im_hsv dimension:
(480, 640, 3)
(y,   x)

therefore,
"""

import numpy as np
import cv2
import time
import sys

# purple_hsv_lows = (3, 111, 115)
# purple_hsv_highs = (14, 246, 255)

hsv_samples = []
im_hsv = None
im_bgr = None


def append_sample(event, x, y, flags, param):
    global hsv_samples, im_hsv, im_bgr
    if event == cv2.EVENT_LBUTTONUP:
        print "x: %s, y: %s" % (x, y)
        hsv = im_hsv[y, x, :].copy()
        hsv_samples.append(hsv)
        print "sample hsv %s" % (hsv,)
        # cv2.circle(img, (x, y), 100, (255, 0, 0), -1)

if __name__ == '__main__':
    # setup callback
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', append_sample)

    # set camera
    camera = cv2.VideoCapture(1)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        # convert to hsv
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

        # display the resulting frame
        cv2.imshow('frame', im_bgr)
        key = cv2.waitKey(10)
        if cv2.waitKey(20) & 0xFF == 27:
            break

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()


# def draw_circle(event, x, y, flags, param):
#     global hsv_samples
#     # mouse callback function
#     if event == cv2.EVENT_LBUTTONUP:
#         cv2.circle(img, (x, y), 100, (255, 0, 0), -1)

# create a black image, a window and bind the function to window
# img = np.zeros((512, 512, 3), np.uint8)
# cv2.namedWindow('image')
# cv2.setMouseCallback('image', draw_circle)

# while(1):
#     cv2.imshow('image', img)
#     if cv2.waitKey(20) & 0xFF == 27:
#         break
# cv2.destroyAllWindows()
