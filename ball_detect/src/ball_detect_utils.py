"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2
import time
import sys

# Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]

# good old values
# orange_hsv_lows = (0, 146, 120)
# orange_hsv_highs = (16, 255, 255)
# green_hsv_lows = (30, 80, 80)
# green_hsv_highs = (50, 255, 255)

# from fine tuning
# orange_hsv_lows = (2.9998, 110.9023, 115.2693)
# orange_hsv_highs = (12.5524, 245.7831, 254.9564)
# green_hsv_lows = (35.8691, 107.8961, 115.0839)
# green_hsv_highs = (57.0588, 224.1300, 254.9658)

# new default values
orange_hsv_lows = (3, 111, 115)
orange_hsv_highs = (14, 246, 255)
green_hsv_lows = (36, 100, 115)
green_hsv_highs = (57, 224, 255)


def hsv_to_center_radius(im_hsv, hsv_lows, hsv_highs, surpress_when_large=True,
                         supress_sv=True):
    """
    Detect ball of from bgr image, returns centers and radius for circles
    """

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

    if supress_sv:
        # elimate v that are smaller than global mean
        v_mean = np.mean(im_hsv[:, :, 2])
        centers_new = []
        radiuses_new = []
        im_mean_mask = np.zeros(im_hsv.shape[:2]).astype(np.uint8)
        for center, radius in zip(centers, radiuses):
            # reset
            im_mean_mask[:] = 0
            # mask to "1" at the ball location
            cv2.circle(im_mean_mask, center, radius, color=1, thickness=-1)
            # area
            area = np.sum(im_mean_mask)
            # get mean, element wise product
            im_mean_mask = im_mean_mask * im_hsv[:, :, 2]
            v_mean_local = np.sum(im_mean_mask) / float(area)
            if v_mean_local >= v_mean * 1:
                centers_new.append(center)
                radiuses_new.append(radius)
        # print len(radiuses), len(radiuses_new)
        centers = centers_new
        radiuses = radiuses_new

    return (centers, radiuses)


def plot_center_radius(im, centers, radiuses, color="orange"):
    """
    Plot circles of centers and radius to im
    """
    # plot center and radius
    for center, radius in zip(centers, radiuses):
        if radius > 2:
            if color == "orange":
                cv2.circle(im, center, radius, (0, 160, 255), 2)
            elif color == "green":
                cv2.circle(im, center, radius, (0, 255, 0), 2)
                pass
            else:
                cv2.circle(im, center, radius, (255, 255, 255), 2)
    return im

def get_ball_coordinate(center0, center1, radius0, radius1):
    """
    x: horizontal
    y: depth
    z: vertical
    """
    # constants
    # f = 10.  # focal length, in cm
    # T = 13.5  # baseline, in cm

    # coefficients
    # kx = 50.
    # ky = 100.
    # kz = 0.0005

    # radius
    # radius = (radius0 + radius1) / 2.0  # average radius

    # calculate x, y, z
    # y = f * T * ky / (abs(center0[1] - center1[1]) * radius + 1e-6)
    # x = (center0[0] - center1[0]) * kx * y
    # z = (240 - center1[1]) * kz * y

    x = (center0[0] + center1[0]) / 2.0 - 320.0
    x = center0[0] - 320.0
    y = 100. / ((radius0 + radius1) / 2.0)
    z = (center0[1] + center1[1]) / 2.0 - 240.0
    z = center0[1] - 240.0

    x = x / 100.
    y = y * 0.3
    z = z / 100.
    return (x, y, z)


def make_valid(thresholds):
    """
    input: 3 tuple
    """
    thresholds = list(thresholds)
    thresholds[0] = max(0, thresholds[0])
    thresholds[1] = max(0, thresholds[1])
    thresholds[2] = max(0, thresholds[2])
    thresholds[0] = min(179, thresholds[0])
    thresholds[1] = min(255, thresholds[1])
    thresholds[2] = min(255, thresholds[2])
    return tuple(thresholds)


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(1)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        if len(sys.argv) > 1 and sys.argv[1] == '-s':
            save_name = str(int(time.time())) + '.npy'
            np.save(save_name, im_bgr)
            print save_name

        im_bgr = cv2.GaussianBlur(im_bgr,(5,5),0)

        # convert to hsv
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

        # get centers and radiuses
        green_centers, green_radiuses = hsv_to_center_radius(im_hsv,
                                                             hsv_lows=green_hsv_lows,
                                                             hsv_highs=green_hsv_highs)
        # plot center and radius
        im_bgr = plot_center_radius(im_bgr, green_centers, green_radiuses,
                                    color="green")

        # get centers and radiuses
        orange_centers, orange_radiuses = hsv_to_center_radius(im_hsv,
                                                               hsv_lows=orange_hsv_lows,
                                                               hsv_highs=orange_hsv_highs)
        # plot center and radius
        im_bgr = plot_center_radius(im_bgr, orange_centers, orange_radiuses,
                                    color="orange")

        # display the resulting frame
        cv2.imshow('frame', im_bgr)
        key = cv2.waitKey(10)
        if key == 27:
            break

        if len(sys.argv) > 1 and sys.argv[1] == '-s':
            time.sleep(0.2)

        # time.sleep(0.5)

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
