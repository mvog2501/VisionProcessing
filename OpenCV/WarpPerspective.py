import cv2
import numpy as np

img = cv2.imread("OpenCV/resources/cards.jpg")

#Define aspect ratio of rectangle
width,height = 250,350
#Define points of rectangle that you want flat
#If you want to find these, just open paint and move cursor; it will tell you values
ptsOne = np.float32([[139,288],[332,298],[200,34],[376,72]])
#Set where the points are on the rectangle i.e. Top Left
ptsTwo = np.float32([[0,0],[width,0],[0,height],[width,height]])
#Actually does the transformation
matrix = cv2.getPerspectiveTransform(ptsOne,ptsTwo)
imgOutput = cv2.warpPerspective(img,matrix,(width,height))
cv2.imshow("Image",img)
cv2.imshow("Warped Image",imgOutput)

cv2.waitKey(0)