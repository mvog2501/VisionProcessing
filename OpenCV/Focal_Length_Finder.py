import cv2
import numpy as np
import math

objectWidth = 8.5
objectHeight = 11

print("Distance to object")
distance = input()
print("Distance: " + distance + " in")

#Define path
path = "OpenCV/resources/PaperPic.jpg"




def getRectangle(path):
    image = cv2.imread(path)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(gray,30,150)
    contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow("Canny", blurred)
    color = (255,0,0)

    if len(contours)>1:
        bestContour = max(contours, key = cv2.contourArea)

        minRect = cv2.minAreaRect(bestContour)
        box = cv2.boxPoints(minRect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, color)

        point1 = box[0]
        point2 = box[1]

        print("We got 'em")
        cv2.imshow("Image",image)

    else: print("Hmmm")
    
    return(point1[0],point1[1],point2[0],point2[1])


def getPixelsToDistance(objectWidth,objectHeight,x1,x2,y1,y2,distance,image):
    
    width = objectWidth
    #Pixels to inches
    wPix = math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
    multiplier = image.shape[1] / wPix
    inchesW = multiplier * objectWidth
    horFOV = math.degrees( math.atan2( ( .5*inchesW ), int(distance) ) ) * 2
    
    return horFOV



image = cv2.imread(path)

x1,x2,y1,y2 = getRectangle(path)

horizontalFOV = getPixelsToDistance(objectWidth,objectHeight,x1,x2,y1,y2,distance,image)
print(horizontalFOV)

cv2.waitKey(0)