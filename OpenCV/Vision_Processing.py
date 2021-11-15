import cv2
import numpy as np
import math

class Vision:
    print("vision is running")
    #exposure = 0
    color = (255,0,0)


    def empty(self,a):
        pass
    
    def __init__(self):
        # Make trackbars
        cv2.namedWindow("Track Bars")
        cv2.resizeWindow("Track Bars", 1000,500)
        cv2.createTrackbar("Hue Min","Track Bars",0,179,self.empty)
        cv2.createTrackbar("Hue Max","Track Bars",179,179,self.empty)
        cv2.createTrackbar("Saturation Min","Track Bars",0,255,self.empty)
        cv2.createTrackbar("Saturation Max","Track Bars",255,255,self.empty)
        cv2.createTrackbar("Value Min","Track Bars",0,255,self.empty)
        cv2.createTrackbar("Value Max","Track Bars",255,255,self.empty)
        cv2.createTrackbar("Number of Yellow Min","Track Bars",0,100000,self.empty)
        cv2.createTrackbar("Thresh Low", "Track Bars", 0 , 255, self.empty)
        cv2.createTrackbar("Thresh High", "Track Bars", 255 , 255, self.empty)
        cv2.createTrackbar("Exposure","Track Bars", -10,10, self.empty)
        cv2.createTrackbar("Collect Data?","Track Bars",0,1, self.empty)
    
    # Find a ball using color and get it's properties
    def ballDetection(self,frame):
        
        #Get posision of trackbars and assign them to variables
        h_min = cv2.getTrackbarPos("Hue Min","Track Bars")
        h_max = cv2.getTrackbarPos("Hue Max","Track Bars")
        s_min = cv2.getTrackbarPos("Saturation Min","Track Bars")
        s_max = cv2.getTrackbarPos("Saturation Max","Track Bars")
        v_min = cv2.getTrackbarPos("Value Min","Track Bars")
        v_max = cv2.getTrackbarPos("Value Max","Track Bars")
        BTLow = cv2.getTrackbarPos("Thresh Low", "Track Bars")
        BTHigh = cv2.getTrackbarPos("Thresh High", "Track Bars")
        exposure = cv2.getTrackbarPos("Exposure",  "Track Bars")

        
        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        frameGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        frameMask = cv2.inRange(frameHSV,lower,upper)
        frameResult = cv2.bitwise_and(frame,frame,mask=frameMask)
        frameGrayMask = cv2.bitwise_and(frameGray,frameGray, mask=frameMask)

        ret,binary = cv2.threshold(frameGrayMask,BTLow,BTHigh,cv2.THRESH_BINARY)
        inverted = cv2.bitwise_not(binary)
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                
        #Draw a squigly line around the object
        cv2.drawContours(frame,contours,-1,(0,255,255),3)
                
        distance = None
        angleToBall = None
        w = None

        #Find a rectangle that fits around the ball (Thill will be used to find location)
        if len(contours)> 0:
            bestContour = max(contours, key = cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(bestContour)
            cv2.rectangle(frameResult,(x,y),( x + w,y + h ),self.color,3)

            #Get distance to ball
            ballW = 6 #Inches (Could be wrong)
            horFOV = 45 #Degree
            ########################

            frameInches = (frame.shape[1]/w)*ballW

            distance = (.5 * frameInches)/math.tan(.5*horFOV)
            #distance = 148.638*.9881**w

            angle = math.degrees(math.atan2(-(.5*frame.shape[1]-(x+.5*w)),distance))
            angleToBall = angle * (.5 * horFOV / 90)

        cv2.imshow("Result",frameResult)
        cv2.imshow("Binary",binary)
        return(distance,angleToBall,w)

    def visionTargetAngle(self,targetFrame):


        #Get posision of trackbars and assign them to variables
        h_min = cv2.getTrackbarPos("Hue Min","Track Bars")
        h_max = cv2.getTrackbarPos("Hue Max","Track Bars")
        s_min = cv2.getTrackbarPos("Saturation Min","Track Bars")
        s_max = cv2.getTrackbarPos("Saturation Max","Track Bars")
        v_min = cv2.getTrackbarPos("Value Min","Track Bars")
        v_max = cv2.getTrackbarPos("Value Max","Track Bars")
        BTLow = cv2.getTrackbarPos("Thresh Low", "Track Bars")
        BTHigh = cv2.getTrackbarPos("Thresh High", "Track Bars")
        exposure = cv2.getTrackbarPos("Exposure",  "Track Bars")

        
        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])
        frameHSV = cv2.cvtColor(targetFrame,cv2.COLOR_BGR2HSV)
        frameGray = cv2.cvtColor(targetFrame,cv2.COLOR_BGR2GRAY)
        frameMask = cv2.inRange(frameHSV,lower,upper)
        frameResult = cv2.bitwise_and(targetFrame,targetFrame,mask=frameMask)
        frameGrayMask = cv2.bitwise_and(frameGray,frameGray, mask=frameMask)

        ret,binary = cv2.threshold(frameGrayMask,BTLow,BTHigh,cv2.THRESH_BINARY)
        inverted = cv2.bitwise_not(binary)
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                
        #Draw a squigly line around the object
        cv2.drawContours(targetFrame,contours,-1,(0,255,255),3)
                
        distance = None
        localXAngle = None
        localYAngle = None

        #Find a rectangle that fits around the ball (Thill will be used to find location)
        if len(contours)> 0:
            bestContour = max(contours, key = cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(bestContour)
            cv2.rectangle(frameResult,(x,y),( x + w,y + h ),self.color,3)

            #Get distance to ball
            targetW = 39.25 #Inches
            targetH = 17 #Inches
            horFOV = 45 #Degrees
            verFOV = 24 #Degrees
            ########################

            frameInches = frame.shape[0]/h*targetH

            distance = (.5 * frameInches)/math.tan(.5*verFOV)

            localYAngle = math.degrees(math.atan2(-(.5*frame.shape[0]-(y+h)),distance))
            localXAngle = math.degrees(math.atan2(-(.5*frame.shape[1]-(x+.5*w)),distance))


            # pointA = (x+int(.5*w),y)
            #pointB = (x+int(.5*w),y+h)
            #cv2.line(frameMask,pointA,pointB,(0,0,0),3)
            

        cv2.imshow("Result",frameResult)
        cv2.imshow("Binary",binary)
        cv2.imshow("Frame mask",frameMask)
        return(distance,localXAngle,localYAngle)

cap = cv2.VideoCapture(0)

vision = Vision()

while True:
    # Capture frames in the video
    cap.set(cv2.CAP_PROP_EXPOSURE,-  cv2.getTrackbarPos("Exposure",  "Track Bars"))
    ret, frame = cap.read()
    cleanFrame = frame.copy()
    targetFrame = frame.copy()
    cv2.imshow("Target Frame", targetFrame)


    #dist,locat = detectingBall.ballDetection(frame)
    #print(dist, locat)
    visionTarget = vision.ballDetection(frame)
    print(visionTarget[0])

    # creating 'q' as the quit button for the video
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# release the cap object
cap.release()
# close all windows
cv2.destroyAllWindows()



