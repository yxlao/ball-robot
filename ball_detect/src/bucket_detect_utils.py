"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2
import time
import sys

purple_hsv_lows = (3, 111, 115)
purple_hsv_highs = (14, 246, 255)


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(1)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        # display the resulting frame
        cv2.imshow('frame', im_bgr)
        key = cv2.waitKey(10)
        if key == 27:
            break

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
