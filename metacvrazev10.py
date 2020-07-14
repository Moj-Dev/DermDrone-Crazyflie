
import logging
import time
import threading


import numpy as np
import cv2
import cv2.aruco as aruco
import glob

import time
import math

WAIT_TIME = 100
from picamera import PiCamera


#from squaternion import Quaternion
#q = Quaternion(1,0,0,0)









import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger




#URI = 'radio://0/80/2M'
URI = 'usb://0'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


#--------------------------------------------------------------------


cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

marker_gap = 11
end_time=0
start_time=0

while_flag=True;
while_counter=0;

yaw_setpoint=0;

x_pose_setpoint=0;
y_pose_setpoint=0;
z_pose_setpoint=0;



x_ekf=0;
y_ekf=0;
z_ekf=0;

yaw_=0;


vision_yaw=0;

vision_yaw_last=0;








circle_trajectory_yaw4 = [
    ( 1.5,0.9,0.15,0 ),
( 1.5,0.9,0.15,0 ),
( 1.5,0.9,0.20,0 ),
( 1.5,0.9,0.25,0 ),
( 1.5,0.9,0.36,0 ),
( 1.5,0.9,0.39,0 ),
( 1.5,0.9,0.42,0 ),
( 1.5,0.9,0.45,0 ),
( 1.5,0.9,0.48,0 ),
( 1.5,0.9,0.51,0 ),
( 1.5,0.9,0.54,0 ),
( 1.5,0.9,0.57,0 ),
( 1.5,0.9,0.6,0 ),
( 1.5,0.9,0.63,0 ),
( 1.5,0.9,0.66,0 ),
( 1.5,0.9,0.7,0 ),
( 1.5,0.9,0.73,0 ),
( 1.5,0.9,0.78,0 ),
( 1.5,0.9,0.8,0 ),


( 1.5,0.8999999999999999,0.8,0.0 ),
( 1.5393941468920689,0.9008625793625058,0.8,2.508710801393725 ),
( 1.5787127813883322,0.9034486640207864,0.8,5.017421602787462 ),
( 1.6178805358383928,0.9077532968564952,0.8,7.526132404181187 ),
( 1.6568223318052195,0.913768226564194,0.8,10.034843205574912 ),
( 1.6954635239787235,0.92148192346781,0.8,12.543554006968636 ),
( 1.7337300432591016,0.9308796016212146,0.8,15.052264808362374 ),
( 1.771548538735673,0.9419432471505624,0.8,17.5609756097561 ),
( 1.8088465182890623,0.9546516527840585,0.8,20.069686411149824 ),
( 1.845552487547212,0.96898045850297,0.8,22.578397212543564 ),
( 1.8815960869288708,0.9849021982359578,0.8,25.087108013937286 ),
( 1.9169082265118678,1.0023863525072247,0.8,27.59581881533101 ),
( 1.951421218467647,1.021399406937559,0.8,30.104529616724747 ),
( 1.9850689068082095,1.0419049164861385,0.8,32.61324041811846 ),
( 2.0177867941967587,1.0638635753099541,0.8,35.1219512195122 ),
( 2.0495121655789696,1.0872332921069396,0.8,37.63066202090592 ),
( 2.0801842083979043,1.1119692707983915,0.8,40.139372822299656 ),
( 2.109744129162137,1.138024096396019,0.8,42.64808362369339 ),
( 2.1381352661436495,1.1653478258890355,0.8,45.15679442508711 ),
( 2.1653031979894664,1.193888083977069,0.8,47.66550522648084 ),
( 2.1911958480388507,1.2235901634653916,0.8,50.174216027874564 ),
( 2.215763584146084,1.2543971301300259,0.8,52.6829268292683 ),
( 2.2389593138175,1.2862499318517129,0.8,55.19163763066202 ),
( 2.260738574480404,1.3190875118095569,0.8,57.70034843205575 ),
( 2.2810596187108434,1.3528469255173665,0.8,60.20905923344949 ),
( 2.2998834942568687,1.3874634614783532,0.8,62.71777003484321 ),
( 2.3171741187038823,1.4228707652269141,0.8,65.22648083623693 ),
( 2.3328983486389676,1.4590009665197259,0.8,67.73519163763068 ),
( 2.3470260431816055,1.4957848094323476,0.8,70.2439024390244 ),
( 2.359530121759014,1.5331517851119572,0.8,72.75261324041811 ),
( 2.370386616015352,1.5710302669317529,0.8,75.26132404181185 ),
( 2.379574715755299,1.609347647787956,0.8,77.77003484320558 ),
( 2.387076808833929,1.6480304792762286,0.8,80.27874564459931 ),
( 2.3928785149164313,1.6870046124807372,0.8,82.78745644599303 ),
( 2.3969687130429516,1.7261953401059846,0.8,85.29616724738678 ),
( 2.399339562945729,1.7655275396789682,0.8,87.80487804878048 ),
( 2.3999865200776576,1.8049258175471692,0.8,90.31358885017421 ),
( 2.3989083443234698,1.8443146533963486,0.8,92.82229965156795 ),
( 2.3961071023768423,1.8836185450111365,0.8,95.33101045296168 ),
( 2.39158816377887,1.9227621530009253,0.8,97.83972125435541 ),
( 2.385360190625498,1.961670445213658,0.8,100.34843205574913 ),
( 2.377435120963641,2.000268840560683,0.8,102.85714285714288 ),
( 2.367828145907827,2.0384833519769954,0.8,105.3658536585366 ),
( 2.3565576805212087,2.076240728242827,0.8,107.87456445993033 ),
( 2.343645328516782,2.113468594394734,0.8,110.38327526132404 ),
( 2.3291158408464607,2.1500955904570445,0.8,112.89198606271778 ),
( 2.3129970682573817,2.1860515082277265,0.8,115.4006968641115 ),
( 2.295319907906399,2.2212674258564946,0.8,117.90940766550523 ),
( 2.2761182441350827,2.255675839957174,0.8,120.41811846689897 ),
( 2.2554288835187553,2.289210795001099,0.8,122.92682926829269 ),
( 2.2332914843140625,2.3218080097435063,0.8,125.43554006968643 ),
( 2.2097484804403242,2.353405000440591,0.8,127.94425087108016 ),
( 2.1848450001403688,2.3839412006210368,0.8,130.45296167247386 ),
( 2.158628779476777,2.4133580771824326,0.8,132.96167247386762 ),
( 2.131150070829343,2.441599242590042,0.8,135.47038327526136 ),
( 2.10246154656915,2.4686105629628563,0.8,137.97909407665506 ),
( 2.072618198093904,2.4943402618397488,0.8,140.4878048780488 ),
( 2.0416772304180553,2.518739019426819,0.8,142.99651567944252 ),
( 2.0096979525197667,2.5417600671356997,0.8,145.50522648083623 ),
( 1.9767416636549116,2.5633592772315974,0.8,148.01393728223 ),
( 1.9428715358560222,2.5834952474192345,0.8,150.5226480836237 ),
( 1.90815249284142,2.6021293802045493,0.8,153.03135888501743 ),
( 1.8726510855666394,2.6192259568800327,0.8,155.54006968641116 ),
( 1.836435364656692,2.634752205991885,0.8,158.04878048780486 ),
( 1.7995747499636998,2.6486783661577493,0.8,160.55749128919862 ),
( 1.7621398974999374,2.660977743114607,0.8,163.06620209059236 ),
( 1.724202564001347,2.6716267608874924,0.8,165.57491289198606 ),
( 1.6858354693811375,2.680605006980935,0.8,168.08362369337982 ),
( 1.6471121573371283,2.687895271506508,0.8,170.59233449477355 ),
( 1.6081068543800248,2.6934835801714856,0.8,173.10104529616726 ),
( 1.568894327552846,2.697359221065366,0.8,175.60975609756096 ),
( 1.5295497411142436,2.699514765192924,0.8,178.1184668989547 ),
( 1.4901485124604128,2.6999460807144264,0.8,180.62717770034843 ),
( 1.4507661675617858,2.698652340865724,0.8,183.13588850174216 ),
( 1.4114781961915999,2.6956360255430254,0.8,185.6445993031359 ),
( 1.372359907223851,2.690902916549326,0.8,188.15331010452962 ),
( 1.3334862842780069,2.6844620865116013,0.8,190.66202090592336 ),
( 1.2949318419871763,2.6763258814900013,0.8,193.1707317073171 ),
( 1.256770483165254,2.666509897312394,0.8,195.67944250871082 ),
( 1.2190753571468294,2.655032949679612,0.8,198.18815331010455 ),
( 1.1819187195713927,2.641917038098706,0.8,200.69686411149826 ),
( 1.145371793880616,2.6271873037133413,0.8,203.20557491289202 ),
( 1.1095046347941975,2.6108719811121768,0.8,205.71428571428575 ),
( 1.0743859940259584,2.593002344207587,0.8,208.22299651567945 ),
( 1.0400831884976005,2.5736126462884807,0.8,210.7317073170732 ),
( 1.0066619713027352,2.552740054362126,0.8,213.24041811846692 ),
( 0.9741864056685217,2.530424577910828,0.8,215.74912891986065 ),
( 0.9427187421565202,2.5067089922000347,0.8,218.25783972125438 ),
( 0.9123192993381347,2.481638756284866,0.8,220.7665505226481 ),
( 0.8830463481733799,2.4552619258722403,0.8,223.27526132404182 ),
( 0.8549560003145991,2.42762906120563,0.8,225.78397212543555 ),
( 0.8281021005492336,2.3987931301490084,0.8,228.29268292682926 ),
( 0.8025361235878176,2.3688094066557723,0.8,230.801393728223 ),
( 0.7783070753950377,2.3377353648172456,0.8,233.31010452961675 ),
( 0.7554613992529933,2.3056305686938727,0.8,235.81881533101046 ),
( 0.7340428867367119,2.272556558140262,0.8,238.3275261324042 ),
( 0.7140925937725755,2.2385767308429414,0.8,240.83623693379795 ),
( 0.6956487619405557,2.2037562207969548,0.8,243.34494773519165 ),
( 0.678746745171105,2.168161773454206,0.8,245.85365853658539 ),
( 0.6634189419772232,2.1318616177829117,0.8,248.36236933797912 ),
( 0.6496947333515946,2.0949253354833797,0.8,250.87108013937285 ),
( 0.6376004264478347,2.0574237276108076,0.8,253.37979094076653 ),
( 0.6271592041538077,2.019428678860776,0.8,255.88850174216032 ),
( 0.61839108065367,1.9810130197775737,0.8,258.397212543554 ),
( 0.6113128630638196,1.9422503871494712,0.8,260.9059233449477 ),
( 0.6059381192162924,1.90321508285856,0.8,263.4146341463415 ),
( 0.6022771516513591,1.8639819314557093,0.8,265.92334494773525 ),
( 0.6003369778691713,1.8246261367336432,0.8,268.43205574912895 ),
( 0.6001213168783162,1.7852231375730767,0.8,270.9407665505227 ),
( 0.6016305820670604,1.745848463338227,0.8,273.4494773519164 ),
( 0.6048618804109498,1.7065775890988715,0.8,275.9581881533101 ),
( 0.6098090180182834,1.6674857909564904,0.8,278.4668989547039 ),
( 0.6164625120028305,1.6286480017517981,0.8,280.9756097560976 ),
( 0.624809608661037,1.5901386674302462,0.8,283.4843205574913 ),
( 0.6348343079188712,1.5520316043408326,0.8,285.99303135888505 ),
( 0.6465173940014546,1.5143998577417517,0.8,288.50174216027875 ),
( 0.659836472266687,1.4773155617840945,0.8,291.01045296167246 ),
( 0.6747660121322591,1.440849801242008,0.8,293.5191637630662 ),
( 0.6912773960137717,1.4050724752543395,0.8,296.02787456446 ),
( 0.7093389741801519,1.3700521633389633,0.8,298.5365853658537 ),
( 0.7289161254212206,1.3358559939366046,0.8,301.0452961672474 ),
( 0.749971323411118,1.3025495157361584,0.8,303.55400696864115 ),
( 0.7724642086403776,1.2701965720281454,0.8,306.06271777003485 ),
( 0.796351665778773,1.2388591783271399,0.8,308.5714285714286 ),
( 0.8215879063206375,1.2085974034977662,0.8,311.0801393728223 ),
( 0.8481245563542398,1.1794692546121157,0.8,313.5888501742161 ),
( 0.8759107492869823,1.1515305657592918,0.8,316.0975609756097 ),
( 0.904893223348674,1.124834891020227,0.8,318.60627177700354 ),
( 0.9350164236859801,1.0994334018129184,0.8,321.11498257839725 ),
( 0.9662226088523599,1.0753747888048475,0.8,323.62369337979095 ),
( 0.9984519614893542,1.0527051685806172,0.8,326.1324041811847 ),
( 1.0316427029870692,1.0314679952436976,0.8,328.6411149825784 ),
( 1.0657312119040783,1.011703977121729,0.8,331.1498257839721 ),
( 1.1006521459197367,0.9934509987350497,0.8,333.6585365853659 ),
( 1.1363385670851516,0.976744048178017,0.8,336.16724738675964 ),
( 1.1727220701327257,0.9616151500523216,0.8,338.6759581881533 ),
( 1.2097329135983161,0.9480933040808532,0.8,341.1846689895471 ),
( 1.2473001535046724,0.9362044295197862,0.8,343.6933797909408 ),
( 1.2853517793499067,0.9259713154754308,0.8,346.2020905923345 ),
( 1.3238148521403228,0.9174135772210952,0.8,348.7108013937283 ),
( 1.3626156442030133,0.9105476185976864,0.8,351.2195121951219 ),
( 1.4016797805102406,0.9053866005701203,0.8,353.7282229965157 ),
( 1.440932381244688,0.9019404159998197,0.8,356.2369337979094 ),
( 1.480298205332315,0.9002156706816502,0.8,358.74564459930315 ),
( 1.5197017946676854,0.9002156706816502,0.8,360.0 ),
    
( 1.5,0.9,0.76,360.0 ),
( 1.5,0.9,0.72,360.0 ),
( 1.5,0.9,0.68,360.0 ),
( 1.5,0.9,0.64,360.0 ),

    
( 1.5,0.9,0.62,360.0 ),
( 1.5,0.9,0.59,360.0 ),
( 1.5,0.9,0.5,360.0 ),
( 1.5,0.9,0.45,360.0 ),
( 1.5,0.9,0.4,360.0 ),
( 1.5,0.9,0.35,360.0 ),
( 1.5,0.9,0.3,360.0 ),
( 1.5,0.9,0.25,360.0 ),
( 1.5,0.9,0.2,360.0 ),
( 1.5,0.9,0.15,360.0 ),
( 1.5,0.9,0.10,360.0 ),
( 1.5,0.9,0.0,360.0 ),
    
    
    
]
   

####---------------------- CALIBRATION ---------------------------
# termination criteria for the iterative algorithm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# checkerboard of size (7 x 6) is used
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
objp *= 0.20

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
#         cv2.imshow('img',img)
        cv2.waitKey(WAIT_TIME)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
cv2.destroyAllWindows()

#-------------------------------------------------------------------
class Uploader:
    def __init__(self):
        self._is_done = False

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done)

        while not self._is_done:
            time.sleep(0.2)

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=100)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 1.0

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))
            
            #if time.time() > endTime:
            #            break

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break




def _sqrt(a):
    """
    There might be rounding errors making 'a' slightly negative.
    Make sure we don't throw an exception.
    """
    if a < 0.0:
        return 0.0
    return math.sqrt(a)

def send_extpose_rot_matrix(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """
    #qw = _sqrt(1 + rot[0][0] + rot[1][1] + rot[2][2]) / 2
    #qx = _sqrt(1 + rot[0][0] - rot[1][1] - rot[2][2]) / 2
    #qy = _sqrt(1 - rot[0][0] + rot[1][1] - rot[2][2]) / 2
    #qz = _sqrt(1 - rot[0][0] - rot[1][1] + rot[2][2]) / 2

    # Normalize the quaternion
    #ql = math.sqrt(qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2)

    #cf.extpos.send_extpose(x, y, z, qx / ql, qy / ql, qz / ql, qw / ql)
    cf.extpos.send_extpose(x, y, z, 0, 0, 0, 0)



def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    
    time.sleep(1)
    wait_for_position_estimator(cf)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    
    cf.param.set_value('locSrv.extPosStdDev', 0.1)

    #cf.param.set_value('locSrv.extQuatStdDev', 0.01)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    #cf.param.set_value('stabilizer.controller', '2')
    cf.param.set_value('stabilizer.controller', '1')

    
    

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration

def yaw_callback(timestamp, data, logconf2):
    global yaw_
    yaw_cf = data['stabilizer.yaw']
    yaw_ = yaw_cf;
    


def position_callback(timestamp, data, logconf):
    global x_ekf
    global y_ekf
    global z_ekf

    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    #print('pos: ({}, {}, {})'.format(x, y, z))
    x_ekf = x;
    y_ekf = y;
    z_ekf = z;
    


def start_position_printing(cf):
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf2 = LogConfig(name='Stabilizer', period_in_ms=200)
    
    log_conf2.add_variable('stabilizer.yaw', 'float')

    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    cf.log.add_config(log_conf)
    cf.log.add_config(log_conf2)

    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()
    
    log_conf2.data_received_cb.add_callback(yaw_callback)
    log_conf2.start()
    
def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(0.5, 1.0)
    time.sleep(3.0)
    relative = False
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 1.0)
    time.sleep(2)
    commander.stop()    
    
def m_run_sequence(scf, sequence):
    cf = scf.cf

    for position in sequence:
        #print('Setting position {}'.format(position))
        for i in range(4):
            #print("Sending setpoint")
            #print(position[0],position[1],position[2],position[3])

            cf.commander.send_position_setpoint(position[1],
                                                position[0],
                                                position[2],
                                                -position[3])
            time.sleep(0.085)
            #print("Sent setpoint")

            

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    
def scan_cap():
    #####################################################################
    camera = PiCamera()
    camera.vflip = True
    camera.hflip = True
    camera.resolution = (3280, 2464)

    print("Start RPi camera preview.")
    time.sleep(3)

    print("Set up RPi camera and get ready for image capturing.")
    #camera.contrast = 20
    camera.image_effect = 'denoise'
    #camera.iso = 200
    #camera.shutter_speed = 20*1000
    time.sleep(3)
    #####################################################################
    capture=True;
    cap_num =0;
    flight_time = 1.0;
    while (True):
        if capture:
            cap_num = cap_num+1;
            camera.capture('image/image' +str(cap_num)+'.jpg')
            print("captured")
            last_t = int(time.time()*1000.0) ;
            capture = False

        scan_dt = int(time.time()*1000.0) - last_t;
        if (scan_dt > flight_time*1000):
            capture = True;
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    


    
def localization(cf):
###------------------ ARUCO TRACKER ---------------------------
    
    i_marker_counter_loop=0
    final_roll_point_sum=0
    final_height_point_sum=0
    final_alt_point_sum=0
    y_point  =  10.0
    x_point  =  20.0
    z_point  =  10.0
    d=10.0;
    d1=0.95;
    d2=1.5;
    d3=0.0;
    qx=0;
    qy=0;
    qz=0.0;
    qw=0.0;
    ql =1.0;
    end_time=0
    start_time=0
    
    first_run_flag=True;


    cutoff_freq = 0.2;
    cutoff_freq_yaw = 1.0;


    RC = 1.0/(2*3.14*cutoff_freq)
    RC_YAW = 1.0/(2*3.14*cutoff_freq_yaw)

    DT = 0.050


    

    f = open( 'log.txt', 'w' )

    while True:
        ret, frame = cap.read()
        # operations on the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.055, mtx, dist)
            
            max_markers=len(ids)
            #print(max_markers)
            #if max_markers > 4 :
                    #max_markers = 4;
            for i_marker_counter_loop in range (0, max_markers):
                
                detected_id=(int)(ids[i_marker_counter_loop][0])
                
                if detected_id< 1024 and detected_id>-1 :
                    i=i_marker_counter_loop
                    rotM = np.zeros(shape=(3,3))
                    cv2.Rodrigues(rvec[i], rotM, jacobian = 0)
                    ypr = cv2.RQDecomp3x3(rotM)
                    yaw_raw=ypr[0][2] #+ 180
                    roll_point_t    = (int)(100*((tvec[i][0])[0])) 
                    height_point_t  = (int)(100*((tvec[i][0])[1]))
                    alt_point       = (int)(100*((tvec[i][0])[2]))
                    
                    col = detected_id%28
                    row = detected_id/28
                                    
                    roll_point    = (int)(roll_point_t*math.cos(3.14*ypr[0][2]/180) + height_point_t*math.sin(3.14*ypr[0][2]/180))
                    height_point  = (int)(-roll_point_t*math.sin(3.14*ypr[0][2]/180) + height_point_t*math.cos(3.14*ypr[0][2]/180))
                                                
                    final_roll_point_sum   = final_roll_point_sum   + (int)(col*marker_gap) - roll_point   # x[detected_id]
                    final_height_point_sum = final_height_point_sum + (int)(row*marker_gap) - height_point #y[detected_id]
                    final_alt_point_sum    = final_alt_point_sum    + alt_point
   
            y_point_  =  float(final_height_point_sum/max_markers)
            x_point_  =  float(final_roll_point_sum  /max_markers)
            z_point  =  float(final_alt_point_sum/max_markers)
            
            final_height_point_sum = 0;
            final_roll_point_sum   = 0;
            final_alt_point_sum    = 0;
            
            x_point = 32*marker_gap - y_point_
            y_point = 28*marker_gap - x_point_
            
            
            if first_run_flag:
                last_x = x_point;
                last_y = y_point;
                last_z = z_point;
                vision_yaw_last = 0;
                first_run_flag=False;
                
            if abs(x_point-last_x) < 100 :
                x    = x_point
            if abs(y_point -last_y) < 100 :
                y    = y_point
            if abs(z_point-last_z) < 100 :
                z    = z_point
            
            d1 = (last_x + ((DT/(DT + RC))*(x - last_x)))
            d2 = (last_y + ((DT/(DT + RC))*(y - last_y)))
            d3 = (last_z + ((DT/(DT + RC))*(z - last_z)))
            vision_yaw = (vision_yaw_last  + ((DT/(DT + RC_YAW))*(ypr[0][2] - vision_yaw_last)))
            
            last_x = d1;
            last_y = d2;
            last_z = d3;
            vision_yaw_last = vision_yaw;
            #print("x: ",x_point, "   y: ",y_point ,"z: ",z_point, "   yaw:  ",ypr[0][2]  )       
            #f.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % ((int)(d1), (int)(d2), (int)(d3), (int)(ypr[0][2]),(int)(100*y_pose_setpoint),(int)(100*x_pose_setpoint),(int)(100*z_pose_setpoint),(int)(100*y_ekf),(int)(100*x_ekf),(int)(100*z_ekf) ))

            #f.write( ''+ (str)d1 + ',' + (str)(d2) + ',' + (str)(d3) + ', 1 , 1 , 1 , 1 , 1 , 1' + '\n' )
         
            end_time = time.time()
            DT=end_time - start_time
            #print("t:  ", (int)(1000*(DT)))
            start_time = time.time()
            

            
            #print("x: ",x_point, "   y: ",y_point ,"z: ",z_point, "   yaw:  ",ypr[0][2]  )
            #if float(1.0*(d3/100)) > 0.40: 
            d1=float(d1/100)
            d2=float(d2/100)
            d3=float(1.0*(d3/100))
            # euler angles from_eluer(roll, pitch, yaw), default is radians, but set
            # degrees true if giving degrees
            #q = Quaternion.from_euler(0, 0, ypr[0][2], degrees=True)
            
            
                #Q =  euler_to_quaternion(0, 0, math.radians(ypr[0][2]));
                #print("x: ",d1, "   y: ",d2 ,"z: ",d3 , "Yaw", ypr[0][2]   )
                #print d1,d2,d3, ypr[0][2],y_pose_setpoint,x_pose_setpoint,z_pose_setpoint,yaw_setpoint,y_ekf,x_ekf,z_ekf, yaw_ 
            print (vision_yaw,yaw_setpoint, yaw_)
                #qx=Q[0];
                #qy=Q[1];
                #qz=Q[2];
                #qw=Q[3];
                #print (x,y,z,w)
                #ql = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            #print(ql)




        #else:
            #print("no ids")
            #d1=float(0.0)
            #d2=float(0.0)
            #d3=float(0.0)
            # code to show 'No Ids' when no markers are found
            #cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            
        #send_extpose_rot_matrix(cf, x_point, y_point, z_point, 0.0)
            
        #if d1 < 2.0 or d2 > 1.0 :
        cf.extpos.send_extpos(float(d1),float(d2),float(d3))


        #cf.extpos.send_extpose(float(d1),float(d2),float(d3),float(qx/ql),float(qy/ql),float(qz/ql),float(qw/ql))

        #else:
        #    print("no external pose")


        # display the resulting frame
        #cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1
        activate_kalman_estimator(cf)
        activate_high_level_commander(cf)
        activate_mellinger_controller(cf)
        #start_position_printing(cf)
        #duration = upload_trajectory(cf, trajectory_id, figure8)
        #print('The sequence is {:.1f} seconds long'.format(duration))
        reset_estimator(cf)
        t1 = threading.Thread(target=localization, args=(cf,))
        t1.setDaemon(True)
        t3 = threading.Thread(target=start_position_printing,args=(cf,))
        t2 = threading.Thread(target=scan_cap, args=())

        
        #t2 = threading.Thread(target=m_run_sequence, args=(scf,circle_trajectory_yaw4,))
        #t2.setDaemon(True)
        #t3.setDaemon(True)

        
        t1.start()
        t2.start()
        t3.start()
        
        time.sleep(7)
        print("START")

        #run_sequence(cf, trajectory_id, duration)
        #m_run_sequence(scf, takeff_landing)
        #t2.start()
        
#
#         camera = PiCamera()
#         camera.vflip = True
#         camera.hflip = True
#         camera.resolution = (3280, 2464)
#         time.sleep(3)
# 
#         print("Set up RPi camera and get ready for image capturing.")
#         #camera.contrast = 20
#         camera.image_effect = 'denoise'
#         time.sleep(3)
#         cap_num =0;




        for position in circle_trajectory_yaw4:
            for i in range(110):

                x_pose_setpoint = position[0];
                y_pose_setpoint = position[1];
                z_pose_setpoint = position[2];
                yaw_setpoint    = -position[3];
                
                cf.commander.send_position_setpoint(position[1],position[0],position[2],-position[3])
                             
                time.sleep(0.004)
                
        cf.commander.send_stop_setpoint()


       