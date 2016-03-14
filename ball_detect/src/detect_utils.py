"""
To run this:
    $ python ball_detect_utils.py
"""

import numpy as np
import cv2
import time
import sys

orange_color = (0, 160, 255)
green_color = (0, 255, 0)
white_color = (255, 255, 255)

# Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]

# new default values
# orange_hsv_lows = (6, 93, 149)
# orange_hsv_highs = (15, 175, 255)
# green_hsv_lows = (46, 115, 96)
# green_hsv_highs = (52, 164, 237)

# bucket (3f floor)
# [current low] (97, 33, 93)
# [current high] (121, 84, 143)

# green ball (3f floor)
# [current low] (46, 115, 96)
# [current high] (52, 164, 237)

# orange ball (3f floor)
# [current low] (6, 93, 149)
# [current high] (15, 175, 255)

# green ball (1f)
green_hsv_lows = (39, 126, 78)
green_hsv_highs = (46, 189, 248)

# orange ball (1f)
# orange_hsv_lows = (3, 189, 139)
# orange_hsv_highs = (9, 237, 255)
orange_hsv_lows = (7, 158, 64)
orange_hsv_highs = (15, 235, 161)

# green bll (3f white desk)
# green_hsv_lows = (35, 177, 89)
# green_hsv_highs = (38, 210, 131)
# green_hsv_lows = (43, 141, 117)
# green_hsv_highs = (52, 179, 173)
green_hsv_lows = (47, 130, 69)
green_hsv_highs = (55, 218, 184)

# orange bll (3f white desk)
# orange_hsv_lows = (7, 144, 112)
# orange_hsv_highs = (11, 198, 181)
#
# bucket
# bucket_hsv_lows = (78, 7, 70)
# bucket_hsv_highs = (120, 47, 86)
# orange bll (3f white desk)
#orange_hsv_lows = (7, 144, 112)
#orange_hsv_highs = (11, 198, 181)

# bucket (3f)
# bucket_hsv_lows = (149, 47, 94)
# bucket_hsv_highs = (171, 90, 156)
# bucket_hsv_lows = (133, 33, 93)
# bucket_hsv_highs = (166, 63, 127)
bucket_hsv_lows = (132, 33, 39)
bucket_hsv_highs = (166, 89, 127)

# green ball (1f)
#green_hsv_lows = (44, 125, 134)
#green_hsv_highs = (56, 179, 239)

# orange ball (1f)
#orange_hsv_lows = (7, 131, 194)
#orange_hsv_highs = (10, 165, 255)

# arm camera
# orange_hsv_lows = (7, 128, 185)
# orange_hsv_highs = (10, 168, 255)
# green_hsv_lows = (39, 121, 142)
# green_hsv_highs = (52, 178, 246)
orange_hsv_lows = (5, 110, 123)
orange_hsv_highs = (11, 201, 255)
green_hsv_lows = (30, 135, 60)
green_hsv_highs = (50, 195, 190)


def hsv_to_im_mask(im_hsv, hsv_lows, hsv_highs, is_bucket=False):
    if is_bucket:
        # mask by threshold
        im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
        im_mask = cv2.medianBlur(im_mask, 7)
        # erode
        im_mask = cv2.erode(im_mask, None, iterations=2)
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
        # elimate v that are smaller than global mean
        # v_mean = np.mean(im_hsv[:, :, 2])
        # centers_new = []
        # radiuses_new = []
        # im_mean_mask = np.zeros(im_hsv.shape[:2]).astype(np.uint8)
        # for center, radius in zip(centers, radiuses):
        # reset
        #     im_mean_mask[:] = 0
        # mask to "1" at the ball location
        #     cv2.circle(im_mean_mask, center, radius, color=1, thickness=-1)
        # area
        #     area = np.sum(im_mean_mask)
        # get mean, element wise product
        #     im_mean_mask = im_mean_mask * im_hsv[:, :, 2]
        #     v_mean_local = np.sum(im_mean_mask) / float(area)
        #     if v_mean_local >= v_mean * 1:
        #         centers_new.append(center)
        #         radiuses_new.append(radius)
        # print len(radiuses), len(radiuses_new)
        # centers = centers_new
        # radiuses = radiuses_new

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
    cv2.imshow('bucket_mask', im_mask)

    # find countours
    contours, hierarchy = cv2.findContours(
        im_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print (len(contours))

    if len(contours) > 0:
        # find countour with max area
        areas = [cv2.contourArea(cnt) for cnt in contours]
        max_idx = np.argmax(areas)
        cnt = contours[max_idx]

        # find center, radius
        # (x,y),r = cv2.minEnclosingCircle(cnt)
        # x = int(x)
        # y = int(y)
        # r = int(r)

        # old method for centorid
        # M = cv2.moments(cnt)
        # x = int(M['m10']/M['m00'])
        # y = int(M['m01']/M['m00'])
        # r = int(np.sqrt(areas[max_idx] / 3.14))

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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

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
        cv2.putText(im, '%.2f' % d, (x,y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
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


def hsv_to_targets(im_hsv):
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
    camera = cv2.VideoCapture(1)

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

        # get centers and radiuses
        green_centers, green_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                                  hsv_lows=green_hsv_lows,
                                                                  hsv_highs=green_hsv_highs)
        # plot center and radius
        im_bgr = plot_center_radius(im_bgr, green_centers, green_radiuses,
                                    color="green")

        # get centers and radiuses
        orange_centers, orange_radiuses = hsv_to_ball_center_radius(im_hsv,
                                                                    hsv_lows=orange_hsv_lows,
                                                                    hsv_highs=orange_hsv_highs)
        # plot center and radius
        im_bgr = plot_center_radius(im_bgr, orange_centers, orange_radiuses,
                                    color="orange")

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

        # time.sleep(0.5)

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
