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

        x_1 = box[0]
        x_2 = box[3]
        y_1 = box[1]
        y_2 = box[2]

        print("We got 'em")
        print(x_2)

    else: print("Hmmm")
    
    return(x_1[0],x_2[0])


def getPixelsToDistance(objectWidth,objectHeight,x1,x2,y1,y2,distance):
    
    #Pixels to inches
    wPix = math.sqrt(math.pow(x_2-x_1,2) + math.pow(y_2-y_1,2))
    multiplier = image.shape[1] / wPix
    inchesW = multiplier(objectWidth)
    horFOV = math.degrees( math.atan( (1/2(inchesW))/distance ) ) * 2
    
    return horFOV





x1,x2 = getRectangle(path)

print(x2)


cv2.waitKey(0)