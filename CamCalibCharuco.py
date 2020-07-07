import cv2
import time
import cv2.aruco as A
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
board = cv2.aruco.CharucoBoard_create(3,3,.068,.034,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)


#Start capturing images for calibration
cap = cv2.VideoCapture(0)

allCorners = []
allIds = []
decimator = 0
for i in range(200):

    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator+=1

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
try:
    print('calib')
    #cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)

    cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                 [    0., 1000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    print('mtx')
    print(camera_matrix)

    print('dist')
    print(distortion_coefficients0)

    print('rvec')
    print(rotation_vectors)
    print('tvec')

    print(translation_vectors)

except:
    print('error')
    cap.release()

cap.release()
cv2.destroyAllWindows()