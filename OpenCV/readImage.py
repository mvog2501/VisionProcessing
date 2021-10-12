import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)




while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)
    if circles is not None:
        circles = np.round(circles[0,:]).astype("int")
        for (x,y,r) in circles:
            cv2.circle(circles,(x,y),r,(0,255,0),4)
            cv2.rectangle(circles,(x-5,y-5),(x+5,y+5),(0,128,255),-1)

    cannyImg = cv2.Canny(gray,100,100)
    cv2.putText(frame,"Lul git gud",(320,240),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),1)
    success, img = cap.read()
    cv2.imshow("Webcam",cannyImg)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

