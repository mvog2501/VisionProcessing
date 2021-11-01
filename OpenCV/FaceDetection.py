import cv2

faceCascade = cv2.CascadeClassifier("OpenCV/Cascades/haarcascade_ball_start.xml")
img = cv2.imread("OpenCv/resources/Image_Pos_4.png")
imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

cv2.imshow("Gray",imgGray)
faces = faceCascade.detectMultiScale(imgGray,1.1,4)


for (x,y,w,h) in faces:
    cv2.rectangle(img,(x+15,y+15),(x+w-15,y+h-15),(255,0,0),2)
    print("Ball found")

cv2.imshow("Result", img)
cv2.waitKey(0)