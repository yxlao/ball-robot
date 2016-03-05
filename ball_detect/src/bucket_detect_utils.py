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
        hsv = im_hsv[y, x, :].copy()
        # if hsv[0] > 50 and hsv[1] > 5 and hsv[2] > 5:
        #     hsv_samples.append(hsv)
        #     print "[appended] x: %s, y: %s, hsv %s" % (x, y, hsv)
        #     hsv_lows, hsv_highs = hsv_threshold_from_sample(hsv_samples)
        #     print "[current low] %s" % (hsv_lows,)
        #     print "[current high] %s" % (hsv_highs,)
        if hsv[0] > 2 and hsv[1] > 5 and hsv[2] > 5:
            hsv_samples.append(hsv)
            print "[appended] x: %s, y: %s, hsv %s" % (x, y, hsv)
            hsv_lows, hsv_highs = hsv_threshold_from_sample(hsv_samples)
            print "[current low] %s" % (hsv_lows,)
            print "[current high] %s" % (hsv_highs,)


def hsv_threshold_from_sample(hsv_samples):
    """
    returns hsv_lows, hsv_highs
    """
    samples = np.array(hsv_samples)

    if len(samples) == 0:
        return ((0,0,0), (0,0,0))
    else:
        return (tuple(np.min(samples, axis = 0).astype(int)),
                tuple(np.max(samples, axis = 0).astype(int)))


def hsv_to_im_mask(im_hsv, hsv_lows, hsv_highs):
    # mask by threshold
    im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
    im_mask = cv2.medianBlur(im_mask, 5)
    # erode
    im_mask = cv2.erode(im_mask, None, iterations=2)
    # dilate
    im_mask = cv2.dilate(im_mask, None, iterations=2)
    return im_mask


if __name__ == '__main__':
    # setup callback
    cv2.namedWindow('input')
    cv2.setMouseCallback('input', append_sample)

    # set camera
    camera = cv2.VideoCapture(1)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        # convert to hsv
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

        # get thresholds
        hsv_lows, hsv_highs = hsv_threshold_from_sample(hsv_samples)

        # get mask image
        im_mask = hsv_to_im_mask(im_hsv, hsv_lows, hsv_highs)

        # display the resulting frame
        cv2.imshow('input', im_bgr)
        cv2.imshow('bucket', im_mask)

        # if cv2.waitKey(20) & 0xFF == 27:
        #     break

        key = cv2.waitKey(30)
        if key == 65288:
            print "received backspace"
            if len(hsv_samples) > 0:
                hsv_samples.pop()
                print "removed last sample"
            else:
                print "sample is empty"

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()