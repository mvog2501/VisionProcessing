import cv2
import numpy as np
import math

objectWidth = 8.5
objectHeight = 11

print("Distance to object")
distance = input()
print("Distance: " + distance + " in")

#Define path
path = "OpenCV/resources/Rectangle.jpg"
image = cv2.imread(path)


color = (255,0,0)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray,(5,5),0)
canny = cv2.Canny(gray,30,150)
contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

cv2.imshow("Canny", blurred)


def getPixelsToDistance(objectWidth,objectHeight,image,distance):
    
    #Pixels to inches
    wPix = math.sqrt(math.pow(x_2-x_1,2) + math.pow(y_2-y_1,2))
    multiplier = image.shape[1] / wPix
    inchesW = multiplier(objectWidth)
    horFOV = math.degrees( math.atan( (1/2(inchesW))/distance ) ) * 2
    
    return horFOV

if len(contours)>1:
    bestContour = max(contours, key = cv2.contourArea)

    minRect = cv2.minAreaRect(bestContour)
    box = cv2.boxPoints(minRect)
    box = np.int0(box)
    cv2.drawContours(image, [box], 0, color)

else: print("Hmmm")




cv2.imshow("With boxes",image)
cv2.waitKey(0)