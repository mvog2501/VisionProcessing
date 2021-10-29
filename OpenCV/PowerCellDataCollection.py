import cv2
import numpy as np
import os
import time


cap = cv2.VideoCapture(0)

color = (255,0,255)
#Cropping Width
CW = 25



def empty(a):
    pass

# Make trackbars
cv2.namedWindow("Track Bars")
cv2.resizeWindow("Track Bars", 1000,400)
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
    #print(h_min,h_max,s_min,s_max,v_min,v_max)

    #Mask off fram  to see if there are any balls in scene (Used for sorting images)
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    frameMask = cv2.inRange(frameHSV,lower,upper)
    frameResult = cv2.bitwise_and(frame,frame,mask=frameMask)
    frameGrayMask = cv2.bitwise_and(frameGray,frameGray, mask=frameMask)


    #another test
    frameThreshold = cv2.inRange(frameHSV,(h_min,s_min,v_min),(h_max,s_max,v_max))
    
    countYellow = cv2.countNonZero(frameMask)
    if countYellow > yellowThresh: #choose condition
        #print("Ball Detected")
        ballInScene = True

        
        # Crop the image to the ball using contours
        ret,binary = cv2.threshold(frameGrayMask,BTLow,BTHigh,cv2.THRESH_BINARY)
        inverted = cv2.bitwise_not(binary)
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                

        cv2.drawContours(frame,contours,-1,(0,255,255),3)
                


        if len(contours)> 0:
            bestContour = max(contours, key = cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(bestContour)
            cv2.rectangle(frame,(x - CW,y - CW),( x + CW + w,y + CW + h ),(255,0,0),3)

            croppedSize = ( ( (y+CW+h )-(y-CW) ) * ( (x+CW+w)-(x-CW) ) )
            
            #if ((y+CW+h)-(y-CW)) < frame.shape[0] and ((x+CW+w)-(x-CW)) < frame.shape[1] and cv2.contourArea(bestContour) > 100 and # make sure that no point is going over edge of image:
            
                
        else:
            o=0
            #send it to the shadow realm
            cropFrame = cv2.imread("OpenCV/resources/lambo.png")
        
        if x-CW < 0:
            x = CW
        if y-CW < 0:
            y = CW
        if x+CW+w > frame.shape[1]:
            w = frame.shape[1] - x - CW
        if y+CW+h > frame.shape[0]:
            h = frame.shape[0] - y - CW
  
        frameCrop = frameClean[x-CW:x+w+CW, y-CW:w+h+CW] 


    else:
        ballInScene = False
        cropFrame = cv2.imread("OpenCV/resources/lambo.png")
    
    
    
    
    
    
    
    
    
    
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
