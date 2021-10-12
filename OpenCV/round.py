import cv2
import numpy as np


img = cv2.imread("resources/circleThingy.png")





grayImg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

circles = cv2.HoughCircles(grayImg, cv2.HOUGH_GRADIENT, 1.2, 100)
if circles is not None:
    circles = np.round(circles[0,:]).astype("int")
    for (x,y,r) in circles:
            cv2.circle(circles,(x,y),r,(0,255,0),4)
            cv2.rectangle(circles,(x-5,y-5),(x+5,y+5),(0,128,255),-1)

    


   

cv2.imshow("Circles",circles)
cv2.waitKey(0)

