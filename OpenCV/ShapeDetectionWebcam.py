import cv2
import numpy as np

def empty(a):
    pass

cap = cv2.VideoCapture(0)

def getContours(frame, frameContour):
    contours, hierarchy = cv2.findContours(frame,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)

        #print(area)

        
        if area>500:
            cv2.drawContours(frameContour,cnt,-1,(255,0,0),3)
            peri = cv2.arcLength(cnt,True)
            #print(peri)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            cv2.rectangle(frameContour,(x,y),(x+w,y+h),(0,255,0),2)

            if objCor == 3: objectType = "Tri"
            elif objCor == 4:
                aspRatio = w/float(h)
                if aspRatio >0.95 and aspRatio < 1.05: objectType = "Square"
                else: objectType = "Rect"
            elif objCor == 5: objectType = "Pent"
            elif objCor == 6: objectType = "Hex"
            elif objCor > 6: objectType = "Circle"
            else:objectType = "None"
            cv2.putText(frameContour,objectType,
            (x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,255,255),2)


cv2.namedWindow("Thresholds")
cv2.resizeWindow("Thresholds", 640, 240)
cv2.createTrackbar("Value Low", "Thresholds", 1, 255, empty)


while(True):
      
    # Capture frames in the video
    ret, frame = cap.read()

    frameGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frameBlur = cv2.GaussianBlur(frameGray,(7,7),1)
    frameCanny = cv2.Canny(frameBlur,50,50)

  
    

    lowThresh = cv2.getTrackbarPos("Value Low", "Thresholds")
    #Thresholding
    ret, frameThresh = cv2.threshold(frameGray, lowThresh, 255, cv2.THRESH_BINARY)



    frameContour = frame.copy()
    
    
    getContours(frameCanny, frameContour)
  
  
    # Display the resulting frame
    #cv2.imshow('video', frame)
    cv2.imshow("Canny",frameCanny)
    cv2.imshow("Frame thingy",frameContour)
    cv2.imshow("Bianary Image",frameThresh)
  
    # creating 'q' as the quit 
    # button for the video
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# release the cap object
cap.release()
# close all windows
cv2.destroyAllWindows()