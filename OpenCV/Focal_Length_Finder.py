import cv2

#Define path
image = cv2.imread("Path")


color = (255,0,0)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray,(5,5),0)
canny = cv2.Canny(blurred,30,150)
contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

bestContour = max(contours, key = cv2.contourArea)

minRect = cv2.minAreaRect
box = cv.boxPoints(minRect)
box = np.intp(box)
cv2.drawContours(drawing, [box], 0, color)


cv2.imshow("With boxes",drawing)
cv2.waitKey(0)