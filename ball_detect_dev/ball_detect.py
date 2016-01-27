import numpy as np
import cv2

# hsv_lows = (0, 165, 60)
# hsv_highs = (16, 218, 255)

hsv_lows = (0, 146, 120)
hsv_highs = (16, 255, 255)

def bgr_to_center_radius(im_bgr):
    # resize
    im_bgr = cv2.resize(im_bgr, (640, 480), interpolation=cv2.INTER_CUBIC)

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

    return (centers, radiuses)

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
        for center, radius in zip(centers, radiuses):
            if radius > 2:
                cv2.circle(im_bgr, center, radius, (0, 255, 0), 2)

        # display the resulting frame
        cv2.imshow('frame', im_bgr)
        key = cv2.waitKey(10)
        if key == 27:
            break

    # when everything done, release the camera
    camera.release()
    cv2.destroyAllWindows()
