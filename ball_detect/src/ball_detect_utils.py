"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2

hsv_lows_default = (0, 146, 120)
hsv_highs_default = (16, 255, 255)


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


def get_ball_coordinate(center0, center1, radius0, radius1):
    """
    x: horizontal
    y: depth
    z: vertical
    """

    # constants
    f = 10.  # focal length, in cm
    T = 13.5  # baseline, in cm

    # coefficients
    kx = 50.
    ky = 100.
    kz = 0.0005

    # radius
    radius = (radius0 + radius1) / 2.0  # average radius

    # calculate x, y, z
    y = f * T * ky / (abs(center0[1] - center1[1]) * radius + 1e-6)
    x = (center0[0] - center1[0]) * kx * y
    z = (240 - center1[1]) * kz * y

    return (x, y, z)


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(0)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        # get centers and radiuses
        centers, radiuses = bgr_to_center_radius(im_bgr)

        # plot center and radius
        im_bgr = plot_center_radius(im_bgr, centers, radiuses)

        # display the resulting frame
        cv2.imshow('frame', im_bgr)
        key = cv2.waitKey(10)
        if key == 27:
            break

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
