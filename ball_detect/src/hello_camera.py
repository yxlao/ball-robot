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

    # set camera
    cameras = [cv2.VideoCapture(idx) for idx in camera_indices]

    # init frame names
    frame_names = ['frame_%d' % idx for idx in camera_indices]

    # main loop
    while(True):
        # init im_bgrs
        im_bgrs = []

        # read frame
        for camera in cameras:
            (_, im_bgr) = camera.read()
            im_bgrs.append(im_bgr.copy())


        # display the resulting frame
        for im_bgr, frame_name in zip(im_bgrs, frame_names):
            cv2.imshow(frame_name, im_bgr)
            key = cv2.waitKey(10)
            if key == 27:
                break

    # when everything done, release the camera
    for camera in cameras:
        camera.release()
    cv2.destroyAllWindows()
