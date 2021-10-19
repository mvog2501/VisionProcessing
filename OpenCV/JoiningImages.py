import cv2
import numpy as np
img = cv2.imread('OpenCV/resources/Lenna.png')




#The following two lines are not great methods
#because the images stacked this way have to have
#the same number of channels(rgb) and you can't add on.
#Stacks images horizontally
imgHor = np.hstack((img,img))

#Stack images vertically
imgVer = np.vstack((img,img))

cv2.imshow("Horizontal",imgHor)
cv2.imshow("Vertical",imgVer)

cv2.waitKey(0)