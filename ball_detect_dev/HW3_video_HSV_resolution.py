import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame0 = cap.read()
    # Resize the resolution of image
    frame=cv2.resize(frame0,(160,120),interpolation=cv2.INTER_CUBIC)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h,s,v=cv2.split(hsv)

    # find orange
    for y in range(1,160):
        for x in range(1,120):
            if h[x,y]>3 and h[x,y]<8 and s[x,y]>146 and s[x,y]<255 and v[x,y]>120 and v[x,y]<255:
                h[x,y]=h[x,y]
            else:
                h[x,y]=255
    ret,thresh = cv2.threshold(h,60,255,0)
    #erode
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(20, 20))
    eroded = cv2.erode(thresh,kernel)
    #dilate
    dilated = cv2.dilate(eroded,kernel)
    #find contour
    image,contours,hierarchy = cv2.findContours(dilated, 1, 2)

    cnt = contours[0]
    # Calculte the center of the ball
    (center_x,center_y),radius = cv2.minEnclosingCircle(cnt)

    center_x=center_x*4
    center_y=center_y*4
    radius=radius*4
    
    center = (int(center_x),int(center_y))
    radius = int(radius)
    # Plot the position of the ball
    if radius>2:
        cv2.circle(frame0,center,radius,(0,255,0),2)
        print center_x
        print center_y
    #gray = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',frame0)
    key = cv2.waitKey(10)
    if key == 27:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
