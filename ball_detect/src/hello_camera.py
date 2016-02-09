#!/usr/bin/python

"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2
import sys

if __name__ == '__main__':

    # parse system argument
    if len(sys.argv) == 1:
        print "using default camera 0"
        camera_indices = [0]
    else:
        camera_indices = []
        for c in sys.argv[1:]:
            try:
                camera_indices.append(int(c))
            except:
                print "error arguments"
                raise
    print camera_indices

    # # set camera
    # camera = cv2.VideoCapture(0)
    #
    # # main loop
    # while(True):
    #     # read frame
    #     (_, im_bgr) = camera.read()
    #
    #     # display the resulting frame
    #     cv2.imshow('frame', im_bgr)
    #     key = cv2.waitKey(10)
    #     if key == 27:
    #         break
    #
    # # when everything done, release the camera
    # camera.release()
    # cv2.destroyAllWindows()
