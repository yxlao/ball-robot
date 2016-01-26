import numpy as np
import cv2

hh = np.zeros((480,640,1), np.uint8)

img = cv2.imread('test_ball_hand.jpg')
# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
h,s,v=cv2.split(hsv)

# find orange
for y in range(1,640):
    for x in range(1,480):
        if h[x,y]>3 and h[x,y]<8 and s[x,y]>156 and s[x,y]<188 and v[x,y]>144 and v[x,y]<214:
            hh[x,y]=h[x,y]
        else:
            hh[x,y]=255

cv2.imwrite('HSV_test.png',hh)

ret,thresh = cv2.threshold(hh,60,255,0)
cv2.imwrite('HSV_test_thresh.png',thresh)
