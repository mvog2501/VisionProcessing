import cv2
import numpy as np
import os
import time


cap = cv2.VideoCapture(0)

color = (255,0,255)
#Cropping Width
CW = 25
####################################################
# Before turning write images on:
# Check the name of your profile and set is as "profile"
#  Create folder "trainingData" in documents
#  Create folder "neg, pos, maybe" in "trainingData"
#  Check that you can read image from "trainingData"
#####################################################
profile  = "Laptop 5"
path = "C:/Users/" + profile +"/Documents/trainingData/"
print(path)
numPos = 0
numNeg = 0
numMaybe = 0

numPosShow = 0
def empty(a):
    pass

# Make trackbars
cv2.namedWindow("Track Bars")
cv2.resizeWindow("Track Bars", 1000,500)
cv2.createTrackbar("Hue Min","Track Bars",0,179,empty)
cv2.createTrackbar("Hue Max","Track Bars",179,179,empty)
cv2.createTrackbar("Saturation Min","Track Bars",0,255,empty)
cv2.createTrackbar("Saturation Max","Track Bars",255,255,empty)
cv2.createTrackbar("Value Min","Track Bars",0,255,empty)
cv2.createTrackbar("Value Max","Track Bars",255,255,empty)
cv2.createTrackbar("Number of Yellow Min","Track Bars",0,100000,empty)
cv2.createTrackbar("Thresh Low", "Track Bars", 0 , 255, empty)
cv2.createTrackbar("Thresh High", "Track Bars", 255 , 255, empty)
cv2.createTrackbar("Exposure","Track Bars", -10,10, empty)
cv2.createTrackbar("Collect Data?","Track Bars",0,1, empty)

exposure = 0



# Things that happen every frame 
while(True):
      
    # Capture frames in the video
    cap.set(cv2.CAP_PROP_EXPOSURE,-  exposure)
    ret, frame = cap.read()
    cleanFrame = frame.copy()

    # Get image area
    frameArea = frame.shape[0] * frame.shape[1]
    
    

    #Convert image to grayscale
    frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    
    #Get posision of trackbars and assign them to variables
    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min","Track Bars")
    h_max = cv2.getTrackbarPos("Hue Max","Track Bars")
    s_min = cv2.getTrackbarPos("Saturation Min","Track Bars")
    s_max = cv2.getTrackbarPos("Saturation Max","Track Bars")
    v_min = cv2.getTrackbarPos("Value Min","Track Bars")
    v_max = cv2.getTrackbarPos("Value Max","Track Bars")
    yellowThresh = cv2.getTrackbarPos("Number of Yellow Min","Track Bars")
    BTLow = cv2.getTrackbarPos("Thresh Low", "Track Bars")
    BTHigh = cv2.getTrackbarPos("Thresh High", "Track Bars")
    exposure = cv2.getTrackbarPos("Exposure","Track Bars")
    collectData = cv2.getTrackbarPos("Collect Data?","Track Bars")
    #print(h_min,h_max,s_min,s_max,v_min,v_max)

    #Mask off frame  to see if there are any balls in scene (Used for sorting images)
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    frameMask = cv2.inRange(frameHSV,lower,upper)
    frameResult = cv2.bitwise_and(frame,frame,mask=frameMask)
    frameGrayMask = cv2.bitwise_and(frameGray,frameGray, mask=frameMask)

    #See if there are any objects that are the right color in the scene (If after masking, the image is black, it will be saved as a negative)
    countYellow = cv2.countNonZero(frameMask)
    if countYellow > yellowThresh: #choose condition
        #print("Ball Detected")
        ballInScene = True

        
        # Crop the image to the ball using contours
        ret,binary = cv2.threshold(frameGrayMask,BTLow,BTHigh,cv2.THRESH_BINARY)
        inverted = cv2.bitwise_not(binary)
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                
        #Draw a squigly line around the object
        cv2.drawContours(frame,contours,-1,(0,255,255),3)
                

        #If it finds any contours, it will draw a rectangle and crop to it
        if len(contours)> 0:
            bestContour = max(contours, key = cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(bestContour)
            cv2.rectangle(frame,(x - CW,y - CW),( x + CW + w,y + CW + h ),(255,0,0),3)

            #Set up dimentions of rectangle that will work with cropping
            if y-CW < 0:
                y = CW
            if y+CW+h > frame.shape[0]:
                h = frame.shape[0] - y - CW
            if x+CW+w > frame.shape[1]:
                w = frame.shape[1] - x - CW
            if x-CW < 0:
                x = CW

            #Do the croppping
            cropFrame = cleanFrame[y-CW:y+h+CW, x-CW:x+w+CW] 
            if collectData == 1:
                numPos += 1
                if numPos % 5 == 0:
                    numPosShow += 1
                    cv2.imwrite(path + "pos/Image_Pos_" + str(numPosShow) + ".png", cropFrame)
            
        else:
            
            #Crop the frame to the full image and send it to "Maybe" folder
            cropFrame = cleanFrame[ 0:frame.shape[0], 0:frame.shape[1]]
            if collectData == 1:
                numMaybe = numMaybe + 1
                cv2.imwrite(path + "maybe/Image_Maybe_" + str(numMaybe) + ".png", cropFrame)

        
        
        


    else:
        ballInScene = False
        cropFrame = cleanFrame[ 0:frame.shape[0], 0:frame.shape[1]]
        if collectData == 1:
                numNeg = numNeg + 1
                cv2.imwrite(path + "neg/Image_Neg_" + str(numNeg) + ".png", cropFrame)
    
    
    
    #Show the videos
    cv2.imshow('video', frame)
    #cv2.imshow("Mask",frameResult)
    cv2.imshow("Binary Image",binary)
    cv2.imshow("Cropped",cropFrame)
   
   
    # creating 'q' as the quit button for the video
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# release the cap object
cap.release()
# close all windows
cv2.destroyAllWindows()
