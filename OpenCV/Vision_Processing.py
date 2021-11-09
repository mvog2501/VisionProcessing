import cv2
import numpy as np

class Vision:

    exposure = 0
    color = (255,0,0)


    def empty(self):
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
                

        #If it finds any contours, it will draw a rectangle and crop to it
        if len(contours)> 0:
            bestContour = max(contours, key = cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(bestContour)
            cv2.rectangle(frame,(x,y),( x + w,y + h ),self.color,3)


        distance = "Gotta define this"
        location = "Need to define this one"

        cv2.imshow("Result",frameResult)
        return(distance,location)





cap = cv2.VideoCapture(0)

detectingBall = Vision()

while True:
    # Capture frames in the video
    cap.set(cv2.CAP_PROP_EXPOSURE,-  Vision.exposure)
    ret, frame = cap.read()
    cleanFrame = frame.copy()


    detectingBall.ballDetection(frame)

    # creating 'q' as the quit button for the video
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# release the cap object
cap.release()
# close all windows
cv2.destroyAllWindows()



