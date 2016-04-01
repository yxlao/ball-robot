from __future__ import print_function
import numpy as np
import cv2
import time
import sys
import math
from detect_utils import orange_hsv_lows, orange_hsv_highs
from detect_utils import green_hsv_lows, green_hsv_highs
from detect_utils import get_contour_feature
from detect_utils import hsv_to_im_mask, hsv_to_targets, plot_targets


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(1)

    # open file
    if len(sys.argv) != 2:
        print("must specify output file to append")
        exit(0)
    out_file = open(sys.argv[1], "a")

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()
        im_bgr = cv2.resize(im_bgr, None, fx=0.5, fy=0.5,
                            interpolation=cv2.INTER_CUBIC)
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

        # [green ball] ######################
        # mask by threshold
        im_mask = hsv_to_im_mask(im_hsv, green_hsv_lows, green_hsv_highs)
        # find contours
        contours = cv2.findContours(im_mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        # get and write feature
        for contour in contours:
            feature = get_contour_feature(contour)
            print(*feature, file=out_file)
            print(*feature)

        # [orange ball] ######################
        # mask by threshold
        im_mask = hsv_to_im_mask(im_hsv, orange_hsv_lows, orange_hsv_highs)
        # find contours
        contours = cv2.findContours(im_mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        # get and write feature
        for contour in contours:
            feature = get_contour_feature(contour)
            print(*feature, file=out_file)
            print(*feature)

        # draw targests
        targets = hsv_to_targets(im_hsv)
        im_bgr = plot_targets(im_bgr, targets)

        # display the resulting frame
        cv2.imshow('frame', im_bgr)
        key = cv2.waitKey(10)
        if key == 27:
            break

        if len(sys.argv) > 1 and sys.argv[1] == '-s':
            time.sleep(0.2)

        time.sleep(0.02)

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
    out_file.close()
