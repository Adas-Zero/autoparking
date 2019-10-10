import numpy as np
import cv2
import glob
import sys
import os, shutil # directory management
import atexit

# parameter
tarCnt = 10 # target image to take for the calibration
winWid = 640 # resize image for debug purpose
winHei = 360
normalLens = False # not a wide angle camera
fisheyeLens = True # a wide angle camera
CHECKERBOARD = (8, 6) # row by colum, count inner crosspoint

# setting video input
cap = cv2.VideoCapture(0) # if /dev/video0 then 0

# clean up
def cleanup():
    # when everthing done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
    # if nothing done remove directory
    if os.path.exists(path) and len(os.listdir(path)) == 0:
        os.rmdir(path)
    print("Bye")
    
atexit.register(cleanup)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(7,5,0)
#objp = np.zeros((6*8,3), np.float32)
#objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space 
imgpoints = [] # 3d points in image plane.

# Create a directory where to store relavant data
path = os.path.dirname(os.path.abspath(__file__))
dir_ind = 0
while os.path.exists(os.path.join(path, "camcalib%s" % dir_ind)):
    dir_ind += 1
path = os.path.join(path, "camcalib%s" % dir_ind)
os.mkdir(path)

imgCnt = 0
find = False
while(True):
    ret, frame = cap.read()
    _img_shape = frame.shape[:2]
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # display the resulting frame
    cv2.imshow('frame', cv2.resize(gray, (winWid, winHei)))

    # user input to find another chessboard pattern
    if cv2.waitKey(1) & 0xFF == ord ('n'):
        find = True
    
    # user input for cancle and remove all data
    if cv2.waitKey(1) & 0xFF == ord ('q'):
        for the_file in os.listdir(path):
            file_path = os.path.join(path, the_file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path): shutil.rmtree(file_path)
            except Exception as e:
                print(e)
        sys.exit(0)

    # user input for quit and keep all data
    if cv2.waitKey(1) & 0xFF == ord ('s'):
        break

    # Find the chess board corners every frame interval
    if find:
        find = False

        if normalLens:
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,None)
        elif fisheyeLens:
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            if normalLens:
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            elif fisheyeLens:
                corners2 = cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)

            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2,ret)
            imgName = 'chessBoardCorners_{:02d}.jpg'.format(imgCnt)
            print(imgName, 'taken.')
            cv2.imshow('chessBoardCorners', cv2.resize(img, (winWid, winHei)))

            # Write chessboard image to path
            cv2.imwrite(path+'/'+imgName, img)
            imgCnt += 1

    # Stop if we find enough pattern image
    if imgCnt == tarCnt:
        break

# when everthing done, release the capture
cap.release()
cv2.destroyAllWindows()

# prompt the result and ask whether to proceed
print("Required chess board {:d} all found".format(imgCnt))

if normalLens: 
    print("Calibrating normal lens..\n\n")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    print("ret", ret)
    print("mtx", mtx)
    print("dist", dist)
    print("rvecs", rvecs)
    print("tvecs", tvecs)
    
    # print error
    tot_error = 0
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error += error
    print("total error: ", mean_error/len(objpoints))

    calibrateCamera(objpoints, imgpoints, gray)

    # testing undistortion 
    print("Testing undistortion")
    test_path = "testUndistort"
    for idx, the_file in enumerate(os.listdir(test_path)):
        file_path = os.path.join(test_path, the_file)
        print(file_path, "loaded")
        
        # load test image
        img = cv2.imread(file_path)
        cv2.imshow('testImage', cv2.resize(img, (winWid, winHei)))
        h, w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    
        # undistort
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        
        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imwrite('calibResult_{:02d}.png'.format(idx), dst)
       	cv2.imshow('undistorted', dst) 
        cv2.waitKey(0)


calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")

DIM=_img_shape[::-1]
K=np.array(K.tolist())
D=np.array(D.tolist())

def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", cv2.resize(undistorted_img, (winWid, winHei)))


# testing undistortion 
print("Testing undistortion")
test_path = "testUndistort"
for idx, the_file in enumerate(os.listdir(test_path)):
    file_path = os.path.join(test_path, the_file)
    print(file_path, "loaded")
    
    # load test image
    img = cv2.imread(file_path)
    cv2.imshow('testImage', cv2.resize(img, (winWid, winHei)))
    undistort(file_path)
    cv2.waitKey(0)

print("Process finished")
