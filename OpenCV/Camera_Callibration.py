import cv2
import numpy as np
import glob



checkerboard = (6,9)#size
criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

#Create a vecotr to store vecots of 3D points for each checkerboard image
objpoints = []
#Vector to store 2D points
imgpoints = []

objp = np.zeros((1, checkerboard[0] * checkerboard[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:checkerboard[0],0:checkerboard[1]].T.reshape(-1,2)
prev_img_shape = None

images = glob.glob('OpenCV/checkerboards/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #Find the corners
    ret, corners = cv2.findChessboardCorners(gray,checkerboard,cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    #If criterion is met, refine the corners
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1),criteria)

        imgpoints.append(corners2)
        

        img = cv2.drawChessboardCorners(img, checkerboard,corners2,ret)
    else:
        print("Ret not true")

    cv2.imshow("Img",img)
    cv2.waitKey(0)
cv2.destroyAllWindows()

h,w = img.shape[:2]

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix : \n")
print(mtx)

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

dst = cv2.undistort(img,mtx,dist,None,newcameramtx)

x,y,w,h = roi
dst = dst[y:y+h,x:x+w]
cv2.imwrite("OpenCV/resources/calibrated.png",dst)