import numpy as np
import cv2
import time
import sys
import math
from detect_utils import orange_hsv_lows, orange_hsv_highs
from detect_utils import green_hsv_lows, green_hsv_highs
from detect_utils import get_contour_feature
from detect_utils import hsv_to_im_mask, hsv_to_targets, plot_targets


def im_mask_to_center_radius(im_mask, surpress_when_large=True, supress_paper=True):
    # find contours
    contours = cv2.findContours(im_mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    # contours = tuple(contours)

    # get centers and radiuses
    centers = []
    radiuses = []

    if len(contours) > 0:
        # fit circles
        for contour in contours:
            center, radius = cv2.minEnclosingCircle(contour)
            centers.append((int(center[0]), int(center[1])))  # a tuple
            radiuses.append(int(radius))  # an int
        # print len(contours), len(centers), len(radiuses)

        # sort all by largest radius
        sorted_inds = np.argsort(radiuses).tolist()[::-1]
        contours = [contours[ind] for ind in sorted_inds]
        centers = [centers[ind] for ind in sorted_inds]
        radiuses = [radiuses[ind] for ind in sorted_inds]

        # # white area ~= contour area
        # im_sum_mask = np.zeros_like(im_mask).astype(np.uint8)
        # cv2.circle(im_sum_mask, center, radius, color=1., thickness=-1)
        # im_product = im_mask * im_sum_mask
        # white_area = np.sum(im_product) / 255
        # # print "np.max(im_sum_mask)", np.max(im_sum_mask) # 1
        # # print "np.max(im_mask)", np.max(im_mask) # 255
        # # print "np.unique(im_mask)", np.unique(im_mask) # [0, 255]
        # contour_area = cv2.contourArea(contour)
        # print white_area, contour_area, center, radius
        # cv2.imshow('im_mask', im_mask)
        # cv2.imshow('im_sum_mask', im_sum_mask * 255)

        # pick the largest one for now
        # if supress_paper
        # contour = contours[0]
        # center = centers[0]
        # radius = radiuses[0]

        # supress papaer
        if supress_paper:
            for contour, center, radius in zip(contours, centers, radiuses):
                if is_ball(contour):
                    return ([center], [radius])
            return ([], [])

        # supress when large
        if surpress_when_large and len(radiuses) > 0:
            pass
            # max_radius = max(radiuses)
            # if max_radius * 2 > im_mask.shape[0] * 0.4:
            #     centers_new = []
            #     radiuses_new = []
            #     for center, radius in zip(centers, radiuses):
            #         if radius * 2 > im_mask.shape[0] * 0.05:
            #             centers_new.append(center)
            #             radiuses_new.append(radius)
            #     centers = centers_new
            #     radiuses = radiuses_new

    return (centers, radiuses)


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(1)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()
        im_bgr = cv2.resize(im_bgr,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

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

        # time.sleep(0.1)

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
