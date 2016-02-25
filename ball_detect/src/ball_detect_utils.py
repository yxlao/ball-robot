"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2
import time


# Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]
orange_hsv_lows = (0, 146, 120)
orange_hsv_highs = (16, 255, 255)

green_hsv_lows = (30, 80, 80)
green_hsv_highs = (50, 255, 255)


def jaccard(im_mask_pd, im_mask_gt):
    """
    - both input must be bool
    - return between 0 to 1, 1 the best
    """
    assert im_mask_pd.dtype == np.bool
    assert im_mask_gt.dtype == np.bool
    union = float(np.sum(np.logical_or(im_mask_pd, im_mask_gt)))
    interset = float(np.sum(np.logical_and(im_mask_pd, im_mask_gt)))
    if union == 0:
        if interset != 0:
            return -1.0  # more panalty
        else:
            return 0.
    return interset / union


def hsv_to_jaccard(im_hsv, hsv_lows, hsv_highs,
                   im_mask_gt, enable_circular):
    if enable_circular:
        im_mask_pd = hsv_to_circular_bool_mask(im_hsv,
                                               hsv_lows, hsv_highs)
    else:
        im_mask_pd = hsv_to_bool_mask(im_hsv, hsv_lows, hsv_highs)
    return jaccard(im_mask_pd, im_mask_gt)


def threshold_to_score(data,
                       hsv_lows, hsv_highs,
                       color,
                       enable_circular):
    jacards = []

    if color == 'orange':
        for key, d in data.iteritems():
            jacard = hsv_to_jaccard(d['im_hsv'],
                                    hsv_lows, hsv_highs,
                                    d['im_mask_orange'], enable_circular)
            jacards.append(jacard)
    elif color == 'green':
        for key, d in data.iteritems():
            jacard = hsv_to_jaccard(d['im_hsv'],
                                    hsv_lows, hsv_highs,
                                    d['im_mask_green'], enable_circular)
            jacards.append(jacard)
    else:
        raise

    return float(np.sum(jacards))


def hypteropt_ball_objective(args):
    hsv_lows, hsv_highs, color, enable_circular = args
    return -threshold_to_score(data, hsv_lows, hsv_highs,
                               color, enable_circular)


def hsv_to_bool_mask(im_hsv, hsv_lows, hsv_highs):
    """
    return boolean mask
    """
    # mask by threshold
    im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
    im_mask = cv2.medianBlur(im_mask, 5)
    # erode
    im_mask = cv2.erode(im_mask, None, iterations=2)
    # dilate
    im_mask = cv2.dilate(im_mask, None, iterations=2)
    return im_mask.astype(np.bool)


def hsv_to_circular_bool_mask(im_hsv, hsv_lows, hsv_highs, surpress_when_large=True):
    """
    return boolean mask, which is drawn by circles
    """
    centers, radiuses = hsv_to_center_radius(im_hsv, hsv_lows, hsv_highs,
                                             surpress_when_large)
    im_mask = np.zeros(im_hsv.shape[:2])
    for center, radius in zip(centers, radiuses):
        cv2.circle(im_mask, center, radius, 255, -1)

    return im_mask.astype(np.bool)


def hsv_to_center_radius(im_hsv, hsv_lows, hsv_highs, surpress_when_large=True):
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


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(0)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        # save_name = str(int(time.time())) + '.npy'
        # np.save(save_name, im_bgr)
        # print save_name

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

        # time.sleep(1)

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
