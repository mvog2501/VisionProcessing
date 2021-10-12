import cv2

img = cv2.imread("resources/circles.png")

cv2.imshow("Circles window", img)
cv2.waitkey(0)