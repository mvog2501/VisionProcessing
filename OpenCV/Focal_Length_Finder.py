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
    imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(gray,30,150)

    frameMask = cv2.inRange(imageHSV,(0,0,100),(255,20,250))

    ret,binary = cv2.threshold(frameMask,100,255,cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow("Canny", binary)
    color = (255,0,0)

    if len(contours)>1:
        bestContour = max(contours, key = cv2.contourArea)

        minRect = cv2.minAreaRect(bestContour)
        box = cv2.boxPoints(minRect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, color)

        point1 = box[0]
        point2 = box[1]
        point3 = box[2]

        print("We got 'em")
        cv2.imshow("Image",image)

    else: print("Hmmm")
    
    return(point1[0],point1[1],point2[0],point2[1],point3[0],point3[1])


def getPixelsToDistance(objectWidth,objectHeight,x1,x2,y1,y2,x3,y3,distance,image):
    
    #Pixels to inches
    wPix = math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
    hPix = math.sqrt(math.pow(x3-x2,2) + math.pow(y3-y2,2))
    multiplierW = image.shape[1] / wPix
    multiplierH = image.shape[0] / hPix
    inchesW = multiplierW * objectWidth
    inchesH = multiplierH * objectHeight
    horFOV = math.degrees( math.atan2( ( .5*inchesW ), int(distance) ) ) * 2
    verFOV = math.degrees( math.atan2( ( .5*inchesH ), int(distance) ) ) * 2
    
    return horFOV,verFOV



image = cv2.imread(path)

x1,x2,y1,y2,x3,y3 = getRectangle(path)

horizontalFOV,verticalFOV = getPixelsToDistance(objectWidth,objectHeight,x1,x2,y1,y2,x3,y3,distance,image)
print(horizontalFOV)
print(verticalFOV)



cv2.waitKey(0)