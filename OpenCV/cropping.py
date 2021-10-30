import cv2
import numpy as np

img = cv2.imread("OpenCV/resources/cards.jpg")

CW = 25

x,w,y,h = 100,150,30,160

cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),3)

cropImg = img[ y:y+h+CW, x:x+w]

print(y)
print(CW)
print(y+h+CW)
print(img.shape[0])

cv2.imshow("Image",img)
cv2.imshow("Cropped",cropImg)
cv2.waitKey(0)