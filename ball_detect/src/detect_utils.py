"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2
import time
import sys
import math

orange_color = (0, 160, 255)
green_color = (0, 255, 0)
white_color = (255, 255, 255)

# Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]
green_hsv_lows = (40, 63, 77)
green_hsv_highs = (55, 209, 157)

orange_hsv_lows = (6, 164, 133)
orange_hsv_highs = (10, 235, 247)

bucket_hsv_lows = (143, 42, 60)
bucket_hsv_highs = (178, 86, 137)

def hsv_to_im_mask(im_hsv, hsv_lows, hsv_highs, is_bucket=False, is_arm=False):
    if is_bucket:
        # mask by threshold
        im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
        im_mask = cv2.medianBlur(im_mask, 7)
        # erode
        im_mask = cv2.erode(im_mask, None, iterations=2)
        # dilate
        im_mask = cv2.dilate(im_mask, None, iterations=3)
    elif is_arm:
        # mask by threshold
        im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
        im_mask = cv2.medianBlur(im_mask, 9)
        # erode
        # im_mask = cv2.erode(im_mask, None, iterations=2)
        # dilate
        im_mask = cv2.dilate(im_mask, None, iterations=3)
    else:
        # mask by threshold
        im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
        im_mask = cv2.medianBlur(im_mask, 5)
        # erode
        # im_mask = cv2.erode(im_mask, None, iterations=2)
        # dilate
        im_mask = cv2.dilate(im_mask, None, iterations=3)
    return im_mask

def im_mask_to_center_radius(im_mask, surpress_when_large=True, supress_sv=False):
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
        try:
            contours = [contours[ind] for ind in sorted_inds]
        except:
            import ipdb; ipdb.set_trace()
        centers = [centers[ind] for ind in sorted_inds]
        radiuses = [radiuses[ind] for ind in sorted_inds]

        # try:
        #     # contours.sort(key=dict(zip(contours, radiuses)).get)
        #     contours = [contour for (radius, contour) in sorted(zip(radiuses, contours), reverse=True)]
        #     centers = [center for (radius, center) in sorted(zip(radiuses, centers), reverse=True)]
        #     radiuses = sorted(radiuses, reverse=True)
        # except:
        #     import ipdb; ipdb.set_trace()

        # picke the largest one for now
        contour = contours[0]
        try:
            center = centers[0]
        except:
            import ipdb; ipdb.set_trace()
        radius = radiuses[0]

        # # white area ~= countour area
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

        # supress when large
        if surpress_when_large and len(radiuses) > 0:
            max_radius = max(radiuses)
            if max_radius * 2 > im_mask.shape[0] * 0.4:
                centers_new = []
                radiuses_new = []
                for center, radius in zip(centers, radiuses):
                    if radius * 2 > im_mask.shape[0] * 0.05:
                        centers_new.append(center)
                        radiuses_new.append(radius)
                centers = centers_new
                radiuses = radiuses_new

        if supress_sv:
            pass

    return (centers, radiuses)


def hsv_to_ball_center_radius(im_hsv, hsv_lows, hsv_highs,
                              surpress_when_large=True, supress_sv=False):
    """
    Detect ball of from bgr image, returns centers and radius for circles
    """

    # mask by threshold
    im_mask = hsv_to_im_mask(im_hsv, hsv_lows, hsv_highs)

    # return centers radius
    return im_mask_to_center_radius(im_mask, surpress_when_large, supress_sv)


def hsv_to_bucket_target(im_hsv, hsv_lows, hsv_highs):
    """
    when the bucket fill up the whole picture:
        return center as the center of the whole image
    when bucket is not found:
        return [(), ()]
    other wise:
        return [(center_x, center_y), radius]
    """
    im_mask = hsv_to_im_mask(im_hsv, bucket_hsv_lows, bucket_hsv_highs,
                             is_bucket=True)

    im_mask = cv2.medianBlur(im_mask, 23)
    # cv2.imshow('bucket_mask', im_mask)

    # find contours
    contours, hierarchy = cv2.findContours(
        im_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print (len(contours))

    if len(contours) > 0:
        # find contour with max area
        areas = [cv2.contourArea(cnt) for cnt in contours]
        max_idx = np.argmax(areas)
        cnt = contours[max_idx]

        im_height = float(im_mask.shape[0])
        x, y, w, h = cv2.boundingRect(cnt)
        d = 22.72 / (h / im_height)
        x = int(x + w / 2)
        y = int(y + h / 2)
        w = int(w)
        # print d

        return {'x': x, 'y': y, 'w': w, 'h': h, 'd': d}
    else:
        return None


def plot_center_radius(im, centers, radiuses, color=None):
    """
    Plot circles of centers and radius to im
    """
    # plot center and radius
    for center, radius in zip(centers, radiuses):
        if radius > 2:
            if color == "orange":
                cv2.circle(im, center, radius, orange_color, 2)
            elif color == "green":
                cv2.circle(im, center, radius, green_color, 2)
                pass
            else:
                cv2.circle(im, center, radius, white_color, 2)
    return im


def plot_targets(im, targets):
    """
    targets['green'] = {'x': xxx, 'y': xxx, 'size': xxx}
    targets['orange'] = {'x': xxx, 'y': xxx, 'size': xxx}
    targets['bucket'] = {'x': xxx, 'y': xxx, 'size': xxx}
    """
    if targets['green'] is not None:
        cv2.line(im,
                 (targets['green']['x'] - targets['green']['size'],
                  targets['green']['y']),
                 (targets['green']['x'] + targets['green']['size'],
                  targets['green']['y']),
                 color=green_color, thickness=2)
        cv2.line(im,
                 (targets['green']['x'],
                  targets['green']['y'] - targets['green']['size']),
                 (targets['green']['x'],
                  targets['green']['y'] + targets['green']['size']),
                 color=green_color, thickness=2)
        cv2.putText(im, '%.2f' % targets['green']['d'],
                    (targets['green']['x'], targets['green']['y']),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    if targets['orange'] is not None:
        cv2.line(im,
                 (targets['orange']['x'] - targets['orange']['size'],
                  targets['orange']['y']),
                 (targets['orange']['x'] + targets['orange']['size'],
                  targets['orange']['y']),
                 color=orange_color, thickness=2)
        cv2.line(im,
                 (targets['orange']['x'],
                  targets['orange']['y'] - targets['orange']['size']),
                 (targets['orange']['x'],
                  targets['orange']['y'] + targets['orange']['size']),
                 color=orange_color, thickness=2)
        cv2.putText(im, '%.2f' % targets['orange']['d'],
                    (targets['orange']['x'], targets['orange']['y']),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    if targets['bucket'] is not None:
        x = targets['bucket']['x']
        y = targets['bucket']['y']
        w = targets['bucket']['w']
        h = targets['bucket']['h']
        d = targets['bucket']['d']
        half_w = int(w / 2)
        half_h = int(h / 2)
        cv2.rectangle(im,
                      (x - half_w, y - half_h),
                      (x + half_w, y + half_h),
                      (0, 255, 0), 2)
        cv2.putText(im, '%.2f' % d, (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # cv2.line(im,
        #          (targets['bucket']['x'] - targets['bucket']
        #           ['size'], targets['bucket']['y']),
        #          (targets['bucket']['x'] + targets['bucket']
        #           ['size'], targets['bucket']['y']),
        #          color=white_color, thickness=2)
        # cv2.line(im,
        #          (targets['bucket']['x'], targets['bucket']
        #           ['y'] - targets['bucket']['size']),
        #          (targets['bucket']['x'], targets['bucket']
        #           ['y'] + targets['bucket']['size']),
        #          color=white_color, thickness=2)

    return im


def ball_radius_to_dist(radius, im_height):
    """
    returns distance in centimeters
    """
    # print radius, im_height
    return 2.5 / (radius / float(im_height))


def arm_hsv_to_targets(im_hsv,
                       green_hsv_lows=green_hsv_lows,
                       green_hsv_highs=green_hsv_highs,
                       orange_hsv_lows=orange_hsv_lows,
                       orange_hsv_highs=orange_hsv_highs,
                       bucket_hsv_lows=bucket_hsv_lows,
                       bucket_hsv_highs=bucket_hsv_highs):
    """
    hsv_lows, hsv_highs and everything else use default setting for now
    """
    # green ball
    green_centers, green_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                              hsv_lows=green_hsv_lows,
                                                              hsv_highs=green_hsv_highs)
    if len(green_radiuses) > 0:
        green_centers = [c for (r, c) in sorted(zip(green_radiuses, green_centers),
                                                reverse=True)]
        green_radiuses = sorted(green_radiuses, reverse=True)
        green_dict = {'x': green_centers[0][0],
                      'y': green_centers[0][1],
                      'size': green_radiuses[0],
                      'd': ball_radius_to_dist(green_radiuses[0], im_hsv.shape[0])}
    else:
        green_dict = None

    # orange ball
    orange_centers, orange_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                                hsv_lows=orange_hsv_lows,
                                                                hsv_highs=orange_hsv_highs)
    if len(orange_radiuses) > 0:
        orange_centers = [c for (r, c) in sorted(zip(orange_radiuses, orange_centers),
                                                 reverse=True)]
        orange_radiuses = sorted(orange_radiuses, reverse=True)
        orange_dict = {'x': orange_centers[0][0],
                       'y': orange_centers[0][1],
                       'size': orange_radiuses[0],
                       'd': ball_radius_to_dist(orange_radiuses[0], im_hsv.shape[0])}
    else:
        orange_dict = None

    return {'green': green_dict, 'orange': orange_dict, 'bucket': None}


def hsv_to_targets(im_hsv,
                   green_hsv_lows=green_hsv_lows,
                   green_hsv_highs=green_hsv_highs,
                   orange_hsv_lows=orange_hsv_lows,
                   orange_hsv_highs=orange_hsv_highs,
                   bucket_hsv_lows=bucket_hsv_lows,
                   bucket_hsv_highs=bucket_hsv_highs):
    """
    hsv_lows, hsv_highs and everything else use default setting for now
    """
    # green ball
    green_centers, green_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                              hsv_lows=green_hsv_lows,
                                                              hsv_highs=green_hsv_highs)
    if len(green_radiuses) > 0:
        green_centers = [c for (r, c) in sorted(zip(green_radiuses, green_centers),
                                                reverse=True)]
        green_radiuses = sorted(green_radiuses, reverse=True)
        green_dict = {'x': green_centers[0][0],
                      'y': green_centers[0][1],
                      'size': green_radiuses[0],
                      'd': ball_radius_to_dist(green_radiuses[0], im_hsv.shape[0])}
    else:
        green_dict = None

    # orange ball
    orange_centers, orange_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                                hsv_lows=orange_hsv_lows,
                                                                hsv_highs=orange_hsv_highs)
    if len(orange_radiuses) > 0:
        orange_centers = [c for (r, c) in sorted(zip(orange_radiuses, orange_centers),
                                                 reverse=True)]
        orange_radiuses = sorted(orange_radiuses, reverse=True)
        orange_dict = {'x': orange_centers[0][0],
                       'y': orange_centers[0][1],
                       'size': orange_radiuses[0],
                       'd': ball_radius_to_dist(orange_radiuses[0], im_hsv.shape[0])}
    else:
        orange_dict = None

    # bucket
    bucket_dict = hsv_to_bucket_target(im_hsv, bucket_hsv_lows, bucket_hsv_highs)

    return {'green': green_dict, 'orange': orange_dict, 'bucket': bucket_dict}


if __name__ == '__main__':
    # set camera
    camera = cv2.VideoCapture(0)

    # main loop
    while(True):
        # read frame
        (_, im_bgr) = camera.read()

        if len(sys.argv) > 1 and sys.argv[1] == '-s':
            save_name = str(int(time.time())) + '.npy'
            np.save(save_name, im_bgr)
            print save_name

        im_bgr = cv2.GaussianBlur(im_bgr, (5, 5), 0)

        # convert to hsv
        im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

        # # get centers and radiuses
        # green_centers, green_radiuses = hsv_to_ball_center_radius(im_hsv,
        #                                                           hsv_lows=green_hsv_lows,
        #                                                           hsv_highs=green_hsv_highs)
        # # plot center and radius
        # im_bgr = plot_center_radius(im_bgr, green_centers, green_radiuses,
        #                             color="green")

        # get centers and radiuses
        orange_centers, orange_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                                    hsv_lows=orange_hsv_lows,
                                                                    hsv_highs=orange_hsv_highs)
        # plot center and radius
        im_bgr = plot_center_radius(im_bgr, orange_centers, orange_radiuses,
                                    color="orange")

        # draw targests
        # targets = hsv_to_targets(im_hsv)
        # im_bgr = plot_targets(im_bgr, targets)

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
