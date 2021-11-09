import cv2
import numpy as np

class Vision:

    exposure = 0

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



