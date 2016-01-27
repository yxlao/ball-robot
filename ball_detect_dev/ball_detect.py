import numpy as np
import cv2

camera = cv2.VideoCapture(0)
hsv_lows = (0, 153, 72)
hsv_highs = (16, 218, 146)

while(True):
    # read frame
    im_bgr = camera.read()

    # resize
    # im_bgr = cv2.resize(im_bgr, (160, 120), interpolation=cv2.INTER_CUBIC)

    # mask by threshold
    im_mask = cv2.inRange(im_hsv, hsv_lows, hsv_highs)
    # erode
    im_mask = cv2.erode(im_mask, None, iterations=2)
    # dilate
    im_mask = cv2.dilate(im_mask, None, iterations=2)

    # find contours
    _, contours, _ = cv2.findContours(im_mask, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)

    # get centers and radiuses
    centers = []
    radiuses = []

    for countour in contours:
        center, radius = cv2.minEnclosingCircle(countour)
        centers.append((int(center[0]), int(center[1])))  # a tuple
        radiuses.append(int(radius))  # an int

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
