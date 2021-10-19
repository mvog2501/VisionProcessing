import cv2
import numpy as np

def empty(a):
    pass

path = "OpenCV/resources/Lambo.png"
cv2.namedWindow("Track Bars")
cv2.resizeWindow("Track Bars", 640,240)
cv2.createTrackbar("Hue Min","Track Bars",24,179,empty)
cv2.createTrackbar("Hue Max","Track Bars",53,179,empty)
cv2.createTrackbar("Saturation Min","Track Bars",44,255,empty)
cv2.createTrackbar("Saturation Max","Track Bars",255,255,empty)
cv2.createTrackbar("Value Min","Track Bars",37,255,empty)
cv2.createTrackbar("Value Max","Track Bars",255,255,empty)

while True:  
    img = cv2.imread(path)

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min","Track Bars")
    h_max = cv2.getTrackbarPos("Hue Max","Track Bars")
    s_min = cv2.getTrackbarPos("Saturation Min","Track Bars")
    s_max = cv2.getTrackbarPos("Saturation Max","Track Bars")
    v_min = cv2.getTrackbarPos("Value Min","Track Bars")
    v_max = cv2.getTrackbarPos("Value Max","Track Bars")

    print(h_min,h_max,s_min,s_max,v_min,v_max)

    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHSV,lower,upper)
    imgResult = cv2.bitwise_and(img,img,mask=mask)
    


    cv2.imshow("Original",img)
    cv2.imshow("HSV Space",imgHSV)
    cv2.imshow("Mask",mask)
    cv2.imshow("Image Result", imgResult)
    cv2.waitKey(1)
    #Once good valeues are found, you can set them as default