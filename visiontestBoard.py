# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

The MotionCommander uses velocity setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
import datetime

import numpy as np
import cv2
import cv2.aruco as aruco
import glob

import time
import math

WAIT_TIME = 100
from picamera import PiCamera

first_run_flag=True;

cutoff_freq = 0.2;

RC = 1.0/(2*3.14*cutoff_freq)
DT = 0.050




import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

#URI = 'radio://0/80/2M'
URI = 'usb://0'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


#--------------------------------------------------------------------


#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('WithMarvelmind.avi')

width = 640
height = 480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

i_marker_counter_loop=0
final_roll_point_sum=0
final_height_point_sum=0
final_alt_point_sum=0
marker_gap = 11 #13.3
end_time=0
start_time=0
last_x = 0;
last_y = 0;
last_z = 0;



####---------------------- CALIBRATION ---------------------------
#termination criteria for the iterative algorithm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# checkerboard of size (7 x 6) is used
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
objp *= 0.2

# arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# iterating through all calibration images
# in the folder
images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9,7),None)

    # if calibration pattern is found, add object points,
    # image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        # Refine the corners of the detected corners
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,7), corners2,ret)
        #cv2.imshow('img',img)
        cv2.waitKey(WAIT_TIME)
        
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print(mtx)
print(dist)
cv2.destroyAllWindows()
# mtx =np.array([[890.89532897 ,  0. ,        317.9698427 ],
#                [  0.      ,   890.89532897 , 300.65090936],
#                [  0.       ,    0.   ,        1.        ]]);
# dist =np.array([[-1.28294264e+00],
#                 [ 8.19920939e+01],
#                 [ 2.87460334e-02],
#                 [-1.43234296e-02],
#                 [-1.68401408e+02],
#                 [-6.96640716e-01],
#                 [ 5.07760046e+01],
#                 [ 1.60974746e+02],
#                 [ 0.00000000e+00],
#                 [ 0.00000000e+00],
#                 [ 0.00000000e+00],
#                 [ 0.00000000e+00],
#                 [ 0.00000000e+00],
#                 [ 0.00000000e+00]]);
#rvecs =
#tvecs = 
#-------------------------------------------------------------------


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    #with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        #with MotionCommander(scf) as mc:
            #time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions
            #mc.forward(0.8)
            #mc.back(0.8)
            #time.sleep(1)
            #mc.up(0.4)
            #time.sleep(1)
            
            
            ###------------------ ARUCO TRACKER ---------------------------
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Start capturing images...")
    cap_counter=0;
    cap_num=0;
    last_t=0;
    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    
    markerLength = 5.5   # Here, our measurement unit is centimetre.
    markerSeparation = 5.5   # Here, our measurement unit is centimetre.
    board = aruco.GridBoard_create(28, 32, markerLength, markerSeparation, aruco_dict)
    arucoParams = aruco.DetectorParameters_create()

    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX
    f = open( 'log.txt', 'w' )

    
    while (True):
        #time.sleep(0.01)
        end_time = time.time()
        DT=end_time - start_time
        #print("t:  ", (int)(1000*(DT)))
        start_time = time.time()


        ret, frame = cap.read()
        #print ret
        if ret==True:
   
            # operations on the frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # lists of ids and the corners belonging to each id
            #corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)  # First, detect markers
            aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints)
            
            if np.all(ids != None):
                im_with_aruco_board = aruco.drawDetectedMarkers(frame, corners, ids, (0,255,0))
                retval, rvec, tvec = aruco.estimatePoseBoard( corners, ids, board, mtx, dist,np.float32(rvecs),np.float32(tvecs) )

                #retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, mtx, dist )
                #y    = (int)(tvec[0]) + (int)(28*(markerLength+markerSeparation))
                #x    = (int)(tvec[1])
                #z    = (int)(tvec[2])
                
                dst, jacobian = cv2.Rodrigues(rvec)
                #print(len(dst))

                extristics = np.matrix([[dst[0][0],dst[0][1],dst[0][2],tvec[0][0]],
                             [dst[1][0],dst[1][1],dst[1][2],tvec[1][0]],
                             [dst[2][0],dst[2][1],dst[2][2],tvec[2][0]],
                             [0.0, 0.0, 0.0, 1.0]
                ])
                extristics_I = extristics.I  # inverse matrix
                worldPos = [extristics_I[0,3],extristics_I[1,3],extristics_I[2,3]]

                #print(worldPos)
                if first_run_flag:
                    last_x = (int)(worldPos[1]);
                    last_y =(-(int)(worldPos[0]) + (int)(28*(markerLength+markerSeparation))) ;
                    last_z=(int)(worldPos[2]);
                    first_run_flag=False;
                    
                if abs((int)(worldPos[1])-last_x) < 100 :
                    x    = (int)(worldPos[1])
                if abs((-(int)(worldPos[0]) + (int)(28*(markerLength+markerSeparation))) -last_y) < 100 :
                    y    = (-(int)(worldPos[0]) + (int)(28*(markerLength+markerSeparation)))
                if abs((int)(worldPos[2])-last_z) < 100 :
                    z    = (int)(worldPos[2])
                    
                    
                d1 = last_x + ((DT/(DT + RC))*(x - last_x))
                d2 = last_y + ((DT/(DT + RC))*(y - last_y))
                d3 = last_z + ((DT/(DT + RC))*(z - last_z))
                
                last_x = d1;
                last_y = d2;
                last_z = d3;




                    
                #print("x: ",x, "   y: ",y,"z: ",z)
                #print d1,",",d2 ,",",d3 ,",",1,",",1,",",1,",",1,",",1,",",1
                f.write("%s,%s,%s,1,1,1,1,1,1\n" % (d1, d2, d3))


                

            # display the resulting frame
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()


                break
        
    cap.release()
    f.close()
    cv2.destroyAllWindows()
    
        #mc.start_linear_motion(velocity_x_m, velocity_y_m, velocity_z_m)

        #mc.up(0.5)
        #time.sleep(1)

        #mc.down(0.5)
        #time.sleep(1)

        # We can also set the velocity
        #mc.right(0.5, velocity=0.8)
        #time.sleep(1)
        #mc.left(0.5, velocity=0.4)
        #time.sleep(1)

        # We can do circles or parts of circles
        #mc.circle_right(0.5, velocity=0.5, angle_degrees=180)

        # Or turn
        #mc.turn_left(90)
        #time.sleep(1)

        # We can move along a line in 3D space
        #mc.move_distance(-1, 0.0, 0.5, velocity=0.6)
        #time.sleep(1)

        # There is also a set of functions that start a motion. The
        # Crazyflie will keep on going until it gets a new command.

        #mc.start_left(velocity=0.5)
        # The motion is started and we can do other stuff, printing for
        # instance
        #for _ in range(5):
        #    print('Doing other work')
        #    time.sleep(0.2)

        # And we can stop
        #mc.stop()
        
    camera = PiCamera()
    camera.vflip = True
    camera.hflip = True
    camera.resolution = (3280, 2464)

    print("Start RPi camera preview.")
    camera.start_preview()
    time.sleep(5)

    print("Set up RPi camera and get ready for image capturing.")
    camera.contrast = 20
    camera.image_effect = 'denoise'
    camera.iso = 200
    camera.shutter_speed = 20*1000
    for i in range(1, 0, -1):
        print("Image capturing will start in {} seconds".format(i*5))
        time.sleep(5)
        
    camera.capture_sequence(['/home/pi/picamera/image/image_%03d.jpg' % i
                             for i in range (50)])
    camera.stop_preview()

    
#     flight_time = 20.0;
#     DelayFlag=True;
#     last_t = int(time.time()*1000.0) ;
#     cap_num =0;
#     while (DelayFlag):
#         cap_num = cap_num+1;
#         camera.capture('image/image' +str(cap_num)+'.jpg')
#         #time.sleep(0.1)
# 
#         dt = int(time.time()*1000.0) - last_t;
#         print(dt)
#         if (dt > flight_time*1000):
#             DelayFlag = False;
#     


    # We land when the MotionCommander goes out of scope

