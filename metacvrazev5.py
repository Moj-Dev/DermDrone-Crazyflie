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

import numpy as np
import cv2
import cv2.aruco as aruco
import glob

import time
import math

WAIT_TIME = 100
from picamera import PiCamera




import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

#URI = 'radio://0/80/2M'
URI = 'usb://0'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


#--------------------------------------------------------------------


cap = cv2.VideoCapture(0)

i_marker_counter_loop=0
final_roll_point_sum=0
final_height_point_sum=0
final_alt_point_sum=0
marker_gap = 11
end_time=0
start_time=0

while_flag=True;
while_counter=0;


####---------------------- CALIBRATION ---------------------------
# termination criteria for the iterative algorithm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 24, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# checkerboard of size (7 x 6) is used
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objp *= 0.24

# arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# iterating through all calibration images
# in the folder
images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # find the chess board (calibration pattern) corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    # if calibration pattern is found, add object points,
    # image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        # Refine the corners of the detected corners
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
#         cv2.imshow('img',img)
        cv2.waitKey(WAIT_TIME)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
cv2.destroyAllWindows()

#-------------------------------------------------------------------


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(2)

            # There is a set of functions that move a specific distance
            # We can move in all directions
            #mc.forward(0.8)
            #mc.back(0.8)
            #time.sleep(1)
            mc.up(0.2)
            time.sleep(1)
            
            
            ###------------------ ARUCO TRACKER ---------------------------
            while (while_flag):
                ret, frame = cap.read()
                print("cap ret: ",ret)

                # operations on the frame
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # set dictionary size depending on the aruco marker selected
                aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

                # detector parameters can be set here (List of detection parameters[3])
                parameters = aruco.DetectorParameters_create()
                parameters.adaptiveThreshConstant = 10

                # lists of ids and the corners belonging to each id
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                # font for displaying text (below)
                font = cv2.FONT_HERSHEY_SIMPLEX

                # check if the ids list is not empty
                # if no check is added the code will crash
                if np.all(ids != None):

                    # estimate pose of each marker and return the values
                    # rvet and tvec-different from camera coefficients
                    rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.055, mtx, dist)
                    #(rvec-tvec).any() # get rid of that nasty numpy value array error

                    for i in range(0, ids.size):
                        # draw axis for the aruco markers
                        aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.5)

                    # draw a square around the markers
                    aruco.drawDetectedMarkers(frame, corners)


                    # code to show ids of the marker found
                    strg = ''
                    for i in range(0, ids.size):
                        strg += str(ids[i][0])+', '

                    cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
                    
                    max_markers=len(ids)
                    
                    for i_marker_counter_loop in range (0, max_markers):
                        
                        #print("for loop")
                        detected_id=(int)(ids[i_marker_counter_loop][0])
                        #print("detected id: ",detected_id)
                        
                        if detected_id< 1024 and detected_id>-1 :
                            #print("id: ",ids[i_marker_counter_loop][0] , "  x: ", x[detected_id-1],"  y:  ", y[detected_id-1])
                            i=i_marker_counter_loop
                            rotM = np.zeros(shape=(3,3))
                            cv2.Rodrigues(rvec[i], rotM, jacobian = 0)
                            ypr = cv2.RQDecomp3x3(rotM)
                            #print("tvec:  ", (100*tvec[i-1][0]))
                            yaw_feedback=ypr[0][2]#+90
                            yaw_raw=ypr[0][2] #+ 180
                            height_setpoint = (int)(1000*((tvec[i][0])[2]))
                            roll_point_t    = (int)(100*((tvec[i][0])[0])) 
                            height_point_t  = (int)(100*((tvec[i][0])[1]))
                            alt_point       = (int)(100*((tvec[i][0])[2]))
                            
                            col = detected_id%28
                            row = detected_id/28
                            
                            #if col>=7 and col<14:
                            #    col = col+14
                            #elif col>=21 and col<28:
                            #    col = col-14
                            
                            roll_point    = (int)(roll_point_t*math.cos(3.14*ypr[0][2]/180) + height_point_t*math.sin(3.14*ypr[0][2]/180))
                            height_point  = (int)(-roll_point_t*math.sin(3.14*ypr[0][2]/180) + height_point_t*math.cos(3.14*ypr[0][2]/180))
                            
                            #print("tvec0:  ", tvec[i])
                            #print("roll_pnt:  ",roll_point, "   roll_pnt:  ",height_point ,"alt_pnt:  ",alt_point, "   yaw:  ",ypr[0][2]  )
                            
                            final_roll_point_sum   = final_roll_point_sum   + (int)(col*marker_gap) - roll_point   # x[detected_id]
                            final_height_point_sum = final_height_point_sum + (int)(row*marker_gap) - height_point #y[detected_id]
                            final_alt_point_sum    = final_alt_point_sum    + alt_point
                            aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers
                            cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
                            
                            
                    y_point  =  final_height_point_sum/max_markers
                    #self.y_point  = final_height_point_sum/len(ids)
                    #self.y_point  = self.y_point - self.y_point_bias
                    
                    x_point  =  final_roll_point_sum  /max_markers
                    #self.x_point  = final_roll_point_sum  /len(ids)
                    
                    #self.z_point  = final_alt_point_sum/len(ids)
                    z_point  = final_alt_point_sum/max_markers
                    
                    final_height_point_sum = 0;
                    final_roll_point_sum   = 0;
                    final_alt_point_sum    = 0;
                    
                    print("x: ",x_point, "   y: ",y_point ,"z: ",z_point, "   yaw:  ",ypr[0][2]  )       
                    
                    end_time = time.time()
                    DT=end_time - start_time
                    #print("t:  ", (int)(1000*(DT)))
                    start_time = time.time()
                    
                    
                    '''self.yaw_feedback = self.last_yaw_feedback + (( self.DT/(self.DT + self.YAW_RC))*(self.yaw_feedback - self.last_yaw_feedback))                    self.y_point = self.y_point_last + (( self.DT/(self.DT + self.RC))*(self.y_point - self.y_point_last))
                    self.x_point = self.x_point_last + (( self.DT/(self.DT + self.RC))*(self.x_point - self.x_point_last))
                    self.z_point = self.z_point_last + (( self.DT/(self.DT + self.RC))*(self.z_point - self.z_point_last))
                    
                    self.last_yaw_feedback = self.yaw_feedback
                    self.y_point_last = self.y_point
                    self.x_point_last = self.x_point
                    self.z_point_last = self.z_point'''
                    y_setpoint = 250 #190 #277;#27;
                    x_setpoint = 145 # 65 #150#133;
                    y_err = y_point - y_setpoint;
                    x_err = x_point - x_setpoint;
                    
                    if (y_err > -5 and y_err < 5) :
                        y_err = 0;
                        
                    if (x_err > -5 and x_err < 5) :
                        x_err = 0;
                        
                    if (x_err > -5 and x_err < 5 and y_err > -5 and y_err < 5) :
                        print(" meeeeeeeeeeeet" )       

                        while_counter = while_counter + 1;
                        if while_counter > 5:
                            print(" second  meeeeeeeeeeeet" )       

                            while_counter=101;
                            while_flag=False;
                            break

                    else:
                        while_counter = while_counter - 1;
                        if while_counter < 0:
                            while_counter=0;

                        
                    y_out = y_err*0.01;
                    x_out = x_err*0.01;
                    if (y_out > 0.14) :
                        y_out = 0.14;
                    if (y_out < -0.14) :
                        y_out = -0.14;
                        
                    if (x_out > 0.14) :
                        x_out = 0.14;
                    if (x_out < -0.14) :
                        x_out = -0.14;
                    if (ret == False):
                        x_out=0;
                        y_out=0;
                        
                        
                    

                    
                    

                else:
                    y_out=0;
                    x_out=0;
                    print "no ids"
                    # code to show 'No Ids' when no markers are found
                    cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

                mc.start_linear_motion(y_out, x_out, 0)

                # display the resulting frame
                #cv2.imshow('frame',frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if cv2.waitKey(1) & 0xFF == ord('l'):
                    mc.stop()
                    time.sleep(5)
                    break


            cap.release()
            #mc.stop()
            #time.sleep(10)


            #mc.start_linear_motion(velocity_x_m, velocity_y_m, velocity_z_m)
            mc._set_vel_setpoint(0.0, 0.0, 0.0, 0.0)

            camera = PiCamera()
            camera.vflip = True
            camera.hflip = True
            #camera.resolution = (80, 64)
            camera.resolution = (3280, 2464)

            print("Start RPi camera preview.")
            #camera.start_preview()
            time.sleep(3)

            print("Set up RPi camera and get ready for image capturing.")
            camera.contrast = 20
            camera.image_effect = 'denoise'
            camera.iso = 200
            camera.shutter_speed = 20*1000
            for i in range(1, 0, -1):
                print("Image capturing will start in {} seconds".format(i*5))
                time.sleep(3)
    
    
            mc.up(1.4)
            time.sleep(1)

            #mc.down(0.5)
            #time.sleep(1)

            # We can also set the velocity
            #mc.right(0.5, velocity=0.8)
            #time.sleep(1)
            #mc.left(0.5, velocity=0.4)
            #time.sleep(1)

            # We can do circles or parts of circles
            angle_degrees=360;
            radius_m =1.15;
            velocity = 0.18;
            
            distance = 2 * radius_m * math.pi * angle_degrees / 360.0
            flight_time = distance / velocity

            #self.start_circle_left(radius_m, velocity)
        
            circumference = 2 * radius_m * math.pi
            rate = 360.0 * velocity / circumference

            mc._set_vel_setpoint(0.0, -velocity, 0.0, -rate)
            #_set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        
            #time.sleep(flight_time)
            #time.sleep(flight_time)
            DelayFlag=True;
            last_t = int(time.time()*1000.0) ;
            cap_num =0;
            while (DelayFlag):
                cap_num = cap_num+1;
                camera.capture('image/image' +str(cap_num)+'.jpg')

                dt = int(time.time()*1000.0) - last_t;
                print(dt)
                if (dt > flight_time*1000):
                    DelayFlag = False;
            
        
            mc._set_vel_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(2)
            #camera.stop_preview()


            mc.stop()
            #mc.circle_right(0.5, velocity=0.1, angle_degrees=180)

            # Or turn
            #mc.turn_left(90)8
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
            mc.stop()

            # We land when the MotionCommander goes out of scope

