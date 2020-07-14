# calculate the yaw heading based on the position
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


ZONE = 0;
last_ZONE = 0;

MOVE_FLAG=0;


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


cap = cv2.VideoCapture(1)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

marker_gap = 11
end_time=0
start_time=0

while_flag=True;
while_counter=0;

yaw_setpoint=0;

x_pose_setpoint=1.8;
y_pose_setpoint=1.5;
z_pose_setpoint=0;

vx_ekf=0;
vy_ekf=0;
vz_ekf=0;


x_ekf=0;
y_ekf=0;
z_ekf=0;

yaw_=0;





Cx = 1.5+0.3
Cy = 1.5

takeoff = [
( 1.5,0.85,0.25,0.0 ),
( 1.5,0.85,0.3,0.0 ),
( 1.5,0.85,0.35,0.0 ),
( 1.5,0.85,0.4,0.0 ),
( 1.5,0.85,0.45,0.0 ),
( 1.5,0.85,0.5,0.0 ),
( 1.5,0.85,0.55,0.0 ),
( 1.5,0.85,0.65,0.0 ),
( 1.5,0.85,0.75,0.0 ),
( 1.5,0.85,0.75,0.0 ),
( 1.5,0.85,0.75,0.0 ),
( 1.5,0.85,0.75,0.0 ),
( 1.5,0.85,0.75,0.0 ),
( 1.5,0.85,0.75,0.0 ),
( 1.5,0.85,0.75,0.0 ),

( 1.5,0.85,0.85,0.0 ),
( 1.5,0.85,0.95,0.0 ),
( 1.5,0.85,1.00,0.0 ),
( 1.5,0.85,1.1,0.0 ),
( 1.5,0.85,1.15,0.0 ),
( 1.5,0.85,1.2,0.0 ),
( 1.5,0.85,1.25,0.0 ),
( 1.5,0.85,1.3,0.0 ),
( 1.5,0.85,1.35,0.0 ),
( 1.5,0.85,1.4,0.0 ),
( 1.5,0.85,1.45,0.0 ),
( 1.5,0.85,1.4976,0.0 ),
( 1.5,0.85,1.4976,0.0 ),
( 1.5,0.85,1.4976,0.0 ),
( 1.5,0.85,1.4976,0.0 ),
( 1.5,0.85,1.4976,0.0 ),
( 1.5,0.85,1.4976,0.0 ),
( 1.5,0.85,1.4976,0.0 ),


    ]






circle_trajectory_yaw4 = [
( 1.5,0.85,1.4976,0.0 ),
( 1.51660284328,0.850145092346,1.4952,1.00139082058 ),
( 1.53320061509,0.850580325063,1.4928,2.00278164117 ),
( 1.54978824552,0.851305565207,1.4904,3.00417246175 ),
( 1.56636066776,0.852320591247,1.488,4.00556328234 ),
( 1.58291281963,0.853625093136,1.4856,5.00695410292 ),
( 1.59943964515,0.855218672405,1.4832,6.0083449235 ),
( 1.61593609608,0.857100842281,1.4808,7.00973574409 ),
( 1.63239713345,0.859271027843,1.4784,8.01112656467 ),
( 1.64881772911,0.861728566191,1.476,9.01251738526 ),
( 1.66519286727,0.86447270665,1.4736,10.0139082058 ),
( 1.68151754602,0.867502611003,1.4712,11.0152990264 ),
( 1.69778677886,0.870817353742,1.4688,12.016689847 ),
( 1.71399559623,0.874415922354,1.4664,13.0180806676 ),
( 1.73013904703,0.878297217629,1.464,14.0194714882 ),
( 1.74621220012,0.882460053995,1.4616,15.0208623088 ),
( 1.76221014582,0.886903159884,1.4592,16.0222531293 ),
( 1.77812799745,0.891625178115,1.4568,17.0236439499 ),
( 1.79396089278,0.896624666311,1.4544,18.0250347705 ),
( 1.80970399552,0.90190009734,1.452,19.0264255911 ),
( 1.82535249684,0.907449859784,1.4496,20.0278164117 ),
( 1.84090161678,0.913272258424,1.4472,21.0292072323 ),
( 1.85634660573,0.919365514765,1.4448,22.0305980529 ),
( 1.87168274592,0.925727767576,1.4424,23.0319888734 ),
( 1.8869053528,0.932357073458,1.44,24.033379694 ),
( 1.90200977651,0.93925140744,1.4376,25.0347705146 ),
( 1.91699140329,0.946408663595,1.4352,26.0361613352 ),
( 1.93184565689,0.953826655686,1.4328,27.0375521558 ),
( 1.94656799996,0.961503117829,1.4304,28.0389429764 ),
( 1.96115393546,0.969435705193,1.428,29.0403337969 ),
( 1.975599008,0.977621994707,1.4256,30.0417246175 ),
( 1.98989880522,0.986059485808,1.4232,31.0431154381 ),
( 2.00404895914,0.994745601201,1.4208,32.0445062587 ),
( 2.0180451475,1.00367768765,1.4184,33.0458970793 ),
( 2.03188309504,1.01285301677,1.416,34.0472878999 ),
( 2.04555857486,1.02226878589,1.4136,35.0486787204 ),
( 2.05906740969,1.03192211891,1.4112,36.050069541 ),
( 2.07240547315,1.04181006712,1.4088,37.0514603616 ),
( 2.08556869102,1.0519296102,1.4064,38.0528511822 ),
( 2.0985530425,1.06227765703,1.404,39.0542420028 ),
( 2.11135456142,1.07285104674,1.4016,40.0556328234 ),
( 2.12396933747,1.0836465496,1.3992,41.0570236439 ),
( 2.13639351735,1.09466086804,1.3968,42.0584144645 ),
( 2.14862330602,1.10589063766,1.3944,43.0598052851 ),
( 2.16065496779,1.11733242824,1.392,44.0611961057 ),
( 2.17248482751,1.1289827448,1.3896,45.0625869263 ),
( 2.18410927163,1.14083802866,1.3872,46.0639777469 ),
( 2.19552474941,1.15289465853,1.3848,47.0653685675 ),
( 2.20672777388,1.16514895163,1.3824,48.066759388 ),
( 2.21771492301,1.17759716478,1.38,49.0681502086 ),
( 2.22848284068,1.19023549559,1.3776,50.0695410292 ),
( 2.23902823775,1.20306008359,1.3752,51.0709318498 ),
( 2.24934789305,1.2160670114,1.3728,52.0723226704 ),
( 2.25943865437,1.22925230596,1.3704,53.073713491 ),
( 2.2692974394,1.24261193973,1.368,54.0751043115 ),
( 2.27892123671,1.2561418319,1.3656,55.0764951321 ),
( 2.28830710662,1.26983784966,1.3632,56.0778859527 ),
( 2.29745218216,1.28369580947,1.3608,57.0792767733 ),
( 2.3063536699,1.2977114783,1.3584,58.0806675939 ),
( 2.31500885081,1.31188057496,1.356,59.0820584145 ),
( 2.32341508109,1.32619877139,1.3536,60.083449235 ),
( 2.331569793,1.340661694,1.3512,61.0848400556 ),
( 2.33947049563,1.35526492496,1.3488,62.0862308762 ),
( 2.34711477564,1.37000400363,1.3464,63.0876216968 ),
( 2.35450029802,1.38487442782,1.344,64.0890125174 ),
( 2.36162480683,1.39987165526,1.3416,65.090403338 ),
( 2.36848612581,1.41499110494,1.3392,66.0917941586 ),
( 2.37508215913,1.4302281585,1.3368,67.0931849791 ),
( 2.38141089198,1.44557816166,1.3344,68.0945757997 ),
( 2.38747039121,1.46103642566,1.332,69.0959666203 ),
( 2.39325880588,1.47659822865,1.3296,70.0973574409 ),
( 2.39877436789,1.49225881715,1.3272,71.0987482615 ),
( 2.40401539247,1.50801340753,1.3248,72.1001390821 ),
( 2.4089802787,1.52385718743,1.3224,73.1015299026 ),
( 2.41366751002,1.53978531724,1.32,74.1029207232 ),
( 2.41807565469,1.55579293159,1.3176,75.1043115438 ),
( 2.4222033662,1.57187514084,1.3152,76.1057023644 ),
( 2.42604938371,1.58802703255,1.3128,77.107093185 ),
( 2.42961253243,1.60424367302,1.3104,78.1084840056 ),
( 2.43289172396,1.62052010873,1.308,79.1098748261 ),
( 2.43588595665,1.63685136794,1.3056,80.1112656467 ),
( 2.43859431589,1.65323246213,1.3032,81.1126564673 ),
( 2.4410159744,1.66965838757,1.3008,82.1140472879 ),
( 2.44315019245,1.68612412685,1.2984,83.1154381085 ),
( 2.44499631813,1.70262465038,1.296,84.1168289291 ),
( 2.44655378754,1.71915491794,1.2936,85.1182197497 ),
( 2.44782212493,1.73570988024,1.2912,86.1196105702 ),
( 2.44880094287,1.75228448045,1.2888,87.1210013908 ),
( 2.44948994239,1.76887365573,1.2864,88.1223922114 ),
( 2.44988891301,1.78547233878,1.284,89.123783032 ),
( 2.44999773288,1.80207545942,1.2816,90.1251738526 ),
( 2.44981636874,1.81867794609,1.2792,91.1265646732 ),
( 2.44934487601,1.83527472745,1.2768,92.1279554937 ),
( 2.4485833987,1.85186073387,1.2744,93.1293463143 ),
( 2.44753216941,1.86843089903,1.272,94.1307371349 ),
( 2.44619150924,1.88498016146,1.2696,95.1321279555 ),
( 2.44456182772,1.90150346606,1.2672,96.1335187761 ),
( 2.44264362263,1.91799576565,1.2648,97.1349095967 ),
( 2.44043747992,1.93445202255,1.2624,98.1363004172 ),
( 2.43794407346,1.95086721006,1.26,99.1376912378 ),
( 2.43516416488,1.96723631403,1.2576,100.139082058 ),
( 2.43209860332,1.98355433441,1.2552,101.140472879 ),
( 2.4287483252,1.99981628672,1.2528,102.1418637 ),
( 2.42511435387,2.01601720364,1.2504,103.14325452 ),
( 2.42119779935,2.03215213647,1.248,104.144645341 ),
( 2.416999858,2.04821615667,1.2456,105.146036161 ),
( 2.41252181209,2.06420435737,1.2432,106.147426982 ),
( 2.40776502949,2.08011185485,1.2408,107.148817803 ),
( 2.4027309632,2.09593379004,1.2384,108.150208623 ),
( 2.39742115089,2.11166533001,1.236,109.151599444 ),
( 2.3918372145,2.12730166945,1.2336,110.152990264 ),
( 2.38598085969,2.14283803212,1.2312,111.154381085 ),
( 2.37985387531,2.15826967232,1.2288,112.155771905 ),
( 2.37345813291,2.17359187633,1.2264,113.157162726 ),
( 2.36679558611,2.18879996387,1.224,114.158553547 ),
( 2.35986827003,2.20388928953,1.2216,115.159944367 ),
( 2.35267830068,2.21885524414,1.2192,116.161335188 ),
( 2.3452278743,2.23369325625,1.2168,117.162726008 ),
( 2.33751926666,2.24839879346,1.2144,118.164116829 ),
( 2.32955483242,2.26296736388,1.212,119.16550765 ),
( 2.32133700437,2.27739451741,1.2096,120.16689847 ),
( 2.31286829272,2.29167584716,1.2072,121.168289291 ),
( 2.3041512843,2.30580699081,1.2048,122.169680111 ),
( 2.29518864178,2.31978363189,1.2024,123.171070932 ),
( 2.28598310287,2.33360150112,1.2,124.172461752 ),
( 2.27653747948,2.34725637773,1.1976,125.173852573 ),
( 2.26685465683,2.36074409074,1.1952,126.175243394 ),
( 2.25693759262,2.37406052022,1.1928,127.176634214 ),
( 2.24678931609,2.38720159857,1.1904,128.178025035 ),
( 2.23641292712,2.40016331175,1.188,129.179415855 ),
( 2.22581159524,2.4129417005,1.1856,130.180806676 ),
( 2.21498855872,2.42553286157,1.1832,131.182197497 ),
( 2.20394712352,2.43793294889,1.1808,132.183588317 ),
( 2.19269066235,2.45013817477,1.1784,133.184979138 ),
( 2.18122261358,2.46214481101,1.176,134.186369958 ),
( 2.1695464802,2.47394919011,1.1736,135.187760779 ),
( 2.15766582878,2.48554770633,1.1712,136.189151599 ),
( 2.14558428835,2.49693681681,1.1688,137.19054242 ),
( 2.13330554932,2.50811304267,1.1664,138.191933241 ),
( 2.12083336231,2.51907297004,1.164,139.193324061 ),
( 2.10817153705,2.52981325113,1.1616,140.194714882 ),
( 2.0953239412,2.54033060523,1.1592,141.196105702 ),
( 2.08229449913,2.55062181975,1.1568,142.197496523 ),
( 2.06908719081,2.56068375115,1.1544,143.198887344 ),
( 2.05570605049,2.57051332593,1.152,144.200278164 ),
( 2.04215516556,2.5801075416,1.1496,145.201668985 ),
( 2.02843867523,2.5894634675,1.1472,146.203059805 ),
( 2.0145607693,2.59857824582,1.1448,147.204450626 ),
( 2.00052568689,2.60744909237,1.1424,148.205841446 ),
( 1.98633771512,2.61607329747,1.14,149.207232267 ),
( 1.97200118782,2.62444822682,1.1376,150.208623088 ),
( 1.95752048418,2.63257132221,1.1352,151.210013908 ),
( 1.94290002744,2.64044010238,1.1328,152.211404729 ),
( 1.92814428354,2.64805216377,1.1304,153.212795549 ),
( 1.91325775972,2.6554051812,1.128,154.21418637 ),
( 1.8982450032,2.66249690865,1.1256,155.215577191 ),
( 1.88311059972,2.66932517989,1.1232,156.216968011 ),
( 1.86785917221,2.67588790916,1.1208,157.218358832 ),
( 1.85249537933,2.68218309185,1.1184,158.219749652 ),
( 1.83702391407,2.68820880504,1.116,159.221140473 ),
( 1.82144950231,2.69396320812,1.1136,160.222531293 ),
( 1.80577690136,2.69944454337,1.1112,161.223922114 ),
( 1.79001089854,2.70465113648,1.1088,162.225312935 ),
( 1.7741563097,2.70958139705,1.1064,163.226703755 ),
( 1.75821797773,2.71423381909,1.104,164.228094576 ),
( 1.74220077113,2.7186069815,1.1016,165.229485396 ),
( 1.72610958248,2.72269954845,1.0992,166.230876217 ),
( 1.70994932693,2.72651026984,1.0968,167.232267038 ),
( 1.69372494078,2.73003798165,1.0944,168.233657858 ),
( 1.67744137988,2.73328160633,1.092,169.235048679 ),
( 1.66110361817,2.73624015307,1.0896,170.236439499 ),
( 1.64471664613,2.73891271817,1.0872,171.23783032 ),
( 1.6282854693,2.74129848527,1.0848,172.23922114 ),
( 1.6118151067,2.74339672562,1.0824,173.240611961 ),
( 1.59531058934,2.7452067983,1.08,174.242002782 ),
( 1.57877695863,2.74672815042,1.0776,175.243393602 ),
( 1.5622192649,2.74796031725,1.0752,176.244784423 ),
( 1.54564256583,2.74890292242,1.0728,177.246175243 ),
( 1.52905192489,2.74955567802,1.0704,178.247566064 ),
( 1.51245240982,2.74991838465,1.068,179.248956885 ),
( 1.49584909106,2.74999093151,1.0656,180.250347705 ),
( 1.47924704024,2.74977329646,1.0632,181.251738526 ),
( 1.46265132856,2.74926554596,1.0608,182.253129346 ),
( 1.44606702532,2.74846783511,1.0584,183.254520167 ),
( 1.42949919631,2.74738040759,1.056,184.255910987 ),
( 1.4129529023,2.74600359554,1.0536,185.257301808 ),
( 1.3964331975,2.74433781954,1.0512,186.258692629 ),
( 1.37994512795,2.74238358841,1.0488,187.260083449 ),
( 1.36349373007,2.74014149907,1.0464,188.26147427 ),
( 1.34708402907,2.73761223639,1.044,189.26286509 ),
( 1.33072103741,2.73479657296,1.0416,190.264255911 ),
( 1.31440975329,2.73169536884,1.0392,191.265646732 ),
( 1.29815515911,2.72830957132,1.0368,192.267037552 ),
( 1.28196221997,2.72464021461,1.0344,193.268428373 ),
( 1.26583588212,2.72068841955,1.032,194.269819193 ),
( 1.24978107148,2.71645539325,1.0296,195.271210014 ),
( 1.23380269209,2.7119424287,1.0272,196.272600834 ),
( 1.21790562469,2.70715090444,1.0248,197.273991655 ),
( 1.20209472516,2.70208228406,1.0224,198.275382476 ),
( 1.18637482304,2.69673811583,1.02,199.276773296 ),
( 1.17075072011,2.69112003215,1.0176,200.278164117 ),
( 1.15522718886,2.68522974911,1.0152,201.279554937 ),
( 1.13980897108,2.67906906594,1.0128,202.280945758 ),
( 1.12450077638,2.67263986447,1.0104,203.282336579 ),
( 1.10930728075,2.66594410855,1.008,204.283727399 ),
( 1.09423312518,2.65898384344,1.0056,205.28511822 ),
( 1.07928291416,2.65176119522,1.0032,206.28650904 ),
( 1.06446121437,2.64427837009,1.0008,207.287899861 ),
( 1.04977255319,2.63653765375,0.9984,208.289290682 ),
( 1.0352214174,2.62854141065,0.996,209.290681502 ),
( 1.02081225174,2.6202920833,0.9936,210.292072323 ),
( 1.00654945761,2.61179219152,0.9912,211.293463143 ),
( 0.992437391679,2.60304433167,0.9888,212.294853964 ),
( 0.978480364591,2.59405117586,0.9864,213.296244784 ),
( 0.964682639623,2.5848154711,0.984,214.297635605 ),
( 0.951048431396,2.57534003852,0.9816,215.299026426 ),
( 0.93758190458,2.56562777245,0.9792,216.300417246 ),
( 0.924287172628,2.55568163958,0.9768,217.301808067 ),
( 0.911168296518,2.54550467804,0.9744,218.303198887 ),
( 0.898229283509,2.53509999645,0.972,219.304589708 ),
( 0.885474085921,2.524470773,0.9696,220.305980529 ),
( 0.872906599925,2.51362025446,0.9672,221.307371349 ),
( 0.860530664354,2.50255175522,0.9648,222.30876217 ),
( 0.848350059533,2.49126865623,0.9624,223.31015299 ),
( 0.836368506119,2.479774404,0.96,224.311543811 ),
( 0.824589663969,2.46807250952,0.9576,225.312934631 ),
( 0.813017131018,2.45616654725,0.9552,226.314325452 ),
( 0.801654442185,2.44406015393,0.9528,227.315716273 ),
( 0.790505068288,2.43175702756,0.9504,228.317107093 ),
( 0.77957241499,2.41926092623,0.948,229.318497914 ),
( 0.76885982175,2.40657566696,0.9456,230.319888734 ),
( 0.758370560813,2.39370512456,0.9432,231.321279555 ),
( 0.748107836201,2.38065323044,0.9408,232.322670376 ),
( 0.738074782743,2.36742397139,0.9384,233.324061196 ),
( 0.728274465109,2.35402138839,0.936,234.325452017 ),
( 0.718709876883,2.34044957537,0.9336,235.326842837 ),
( 0.709383939638,2.32671267794,0.9312,236.328233658 ),
( 0.700299502054,2.31281489212,0.9288,237.329624478 ),
( 0.69145933904,2.29876046312,0.9264,238.331015299 ),
( 0.682866150892,2.28455368396,0.924,239.33240612 ),
( 0.674522562463,2.27019889421,0.9216,240.33379694 ),
( 0.666431122367,2.25570047865,0.9192,241.335187761 ),
( 0.658594302193,2.24106286593,0.9168,242.336578581 ),
( 0.65101449576,2.22629052721,0.9144,243.337969402 ),
( 0.643694018375,2.21138797483,0.912,244.339360223 ),
( 0.636635106134,2.19635976087,0.9096,245.340751043 ),
( 0.629839915236,2.18121047583,0.9072,246.342141864 ),
( 0.623310521322,2.16594474716,0.9048,247.343532684 ),
( 0.617048918846,2.1505672379,0.9024,248.344923505 ),
( 0.611057020462,2.13508264522,0.9,249.346314325 ),
( 0.605336656439,2.11949569901,0.8976,250.347705146 ),
( 0.599889574107,2.10381116043,0.8952,251.349095967 ),
( 0.594717437318,2.08803382042,0.8928,252.350486787 ),
( 0.589821825941,2.0721684983,0.8904,253.351877608 ),
( 0.585204235377,2.05622004025,0.888,254.353268428 ),
( 0.580866076104,2.04019331786,0.8856,255.354659249 ),
( 0.576808673245,2.02409322659,0.8832,256.35605007 ),
( 0.573033266166,2.00792468436,0.8808,257.35744089 ),
( 0.569541008092,1.99169262995,0.8784,258.358831711 ),
( 0.566332965761,1.97540202158,0.876,259.360222531 ),
( 0.563410119093,1.95905783534,0.8736,260.361613352 ),
( 0.560773360893,1.94266506367,0.8712,261.363004172 ),
( 0.558423496581,1.92622871388,0.8688,262.364394993 ),
( 0.556361243938,1.90975380658,0.8664,263.365785814 ),
( 0.554587232897,1.89324537414,0.864,264.367176634 ),
( 0.553102005342,1.8767084592,0.8616,265.368567455 ),
( 0.551906014947,1.86014811308,0.8592,266.369958275 ),
( 0.550999627037,1.84356939426,0.8568,267.371349096 ),
( 0.550383118475,1.82697736684,0.8544,268.372739917 ),
( 0.550056677577,1.81037709898,0.852,269.374130737 ),
( 0.550020404057,1.79377366136,0.8496,270.375521558 ),
( 0.550274308997,1.77717212562,0.8472,271.376912378 ),
( 0.550818314838,1.76057756283,0.8448,272.378303199 ),
( 0.551652255409,1.74399504192,0.8424,273.379694019 ),
( 0.552775875978,1.72742962814,0.84,274.38108484 ),
( 0.554188833326,1.71088638154,0.8376,275.382475661 ),
( 0.555890695854,1.69437035536,0.8352,276.383866481 ),
( 0.557880943716,1.67788659456,0.8328,277.385257302 ),
( 0.560158968975,1.66144013423,0.8304,278.386648122 ),
( 0.562724075792,1.64503599805,0.828,279.388038943 ),
( 0.565575480634,1.6286791968,0.8256,280.389429764 ),
( 0.568712312519,1.61237472679,0.8232,281.390820584 ),
( 0.572133613278,1.59612756834,0.8208,282.392211405 ),
( 0.575838337848,1.57994268426,0.8184,283.393602225 ),
( 0.579825354594,1.56382501836,0.816,284.394993046 ),
( 0.584093445651,1.54777949389,0.8136,285.396383866 ),
( 0.588641307297,1.53181101209,0.8112,286.397774687 ),
( 0.593467550356,1.51592445064,0.8088,287.399165508 ),
( 0.598570700613,1.50012466222,0.8064,288.400556328 ),
( 0.603949199273,1.484416473,0.804,289.401947149 ),
( 0.609601403433,1.46880468115,0.8016,290.403337969 ),
( 0.615525586584,1.45329405541,0.7992,291.40472879 ),
( 0.621719939141,1.43788933363,0.7968,292.406119611 ),
( 0.62818256899,1.4225952213,0.7944,293.407510431 ),
( 0.634911502073,1.40741639011,0.792,294.408901252 ),
( 0.641904682986,1.39235747656,0.7896,295.410292072 ),
( 0.649159975609,1.3774230805,0.7872,296.411682893 ),
( 0.656675163758,1.36261776376,0.7848,297.413073713 ),
( 0.664447951862,1.34794604874,0.7824,298.414464534 ),
( 0.672475965664,1.33341241701,0.78,299.415855355 ),
( 0.680756752947,1.319021308,0.7776,300.417246175 ),
( 0.689287784281,1.30477711757,0.7752,301.418636996 ),
( 0.698066453798,1.29068419672,0.7728,302.420027816 ),
( 0.707090079987,1.27674685023,0.7704,303.421418637 ),
( 0.716355906513,1.26296933538,0.768,304.422809458 ),
( 0.725861103059,1.24935586061,0.7656,305.424200278 ),
( 0.735602766192,1.23591058426,0.7632,306.425591099 ),
( 0.745577920246,1.2226376133,0.7608,307.426981919 ),
( 0.755783518235,1.20954100204,0.7584,308.42837274 ),
( 0.766216442782,1.19662475096,0.756,309.429763561 ),
( 0.776873507071,1.18389280541,0.7536,310.431154381 ),
( 0.787751455821,1.17134905447,0.7512,311.432545202 ),
( 0.79884696628,1.15899732972,0.7488,312.433936022 ),
( 0.810156649241,1.14684140409,0.7464,313.435326843 ),
( 0.821677050075,1.13488499069,0.744,314.436717663 ),
( 0.83340464979,1.12313174171,0.7416,315.438108484 ),
( 0.8453358661,1.11158524726,0.7392,316.439499305 ),
( 0.857467054525,1.10024903431,0.7368,317.440890125 ),
( 0.869794509502,1.08912656559,0.7344,318.442280946 ),
( 0.882314465517,1.07822123854,0.732,319.443671766 ),
( 0.895023098254,1.06753638428,0.7296,320.445062587 ),
( 0.907916525765,1.05707526657,0.7272,321.446453408 ),
( 0.920990809655,1.04684108085,0.7248,322.447844228 ),
( 0.934241956285,1.03683695322,0.7224,323.449235049 ),
( 0.947665917992,1.02706593952,0.72,324.450625869 ),
( 0.961258594326,1.01753102438,0.7176,325.45201669 ),
( 0.9750158333,1.00823512031,0.7152,326.45340751 ),
( 0.988933432662,0.999181066814,0.7128,327.454798331 ),
( 1.00300714118,0.990371629525,0.7104,328.456189152 ),
( 1.01723265992,0.981809499351,0.708,329.457579972 ),
( 1.0316056436,0.973497291659,0.7056,330.458970793 ),
( 1.04612170189,0.965437545475,0.7032,331.460361613 ),
( 1.06077640073,0.957632722711,0.7008,332.461752434 ),
( 1.07556526375,0.950085207408,0.6984,333.463143255 ),
( 1.09048377358,0.942797305011,0.696,334.464534075 ),
( 1.10552737323,0.935771241667,0.6936,335.465924896 ),
( 1.12069146753,0.929009163539,0.6912,336.467315716 ),
( 1.1359714245,0.922513136155,0.6888,337.468706537 ),
( 1.15136257675,0.916285143777,0.6864,338.470097357 ),
( 1.16686022294,0.91032708879,0.684,339.471488178 ),
( 1.1824596292,0.904640791129,0.6816,340.472878999 ),
( 1.19815603056,0.899227987715,0.6792,341.474269819 ),
( 1.21394463246,0.894090331932,0.6768,342.47566064 ),
( 1.22982061213,0.889229393113,0.6744,343.47705146 ),
( 1.24577912014,0.884646656072,0.672,344.478442281 ),
( 1.26181528184,0.880343520637,0.6696,345.479833102 ),
( 1.27792419886,0.876321301237,0.6672,346.481223922 ),
( 1.29410095062,0.872581226486,0.6648,347.482614743 ),
( 1.31034059581,0.86912443882,0.6624,348.484005563 ),
( 1.32663817389,0.865951994141,0.66,349.485396384 ),
( 1.34298870666,0.863064861496,0.6576,350.486787204 ),
( 1.35938719971,0.860463922781,0.6552,351.488178025 ),
( 1.37582864401,0.858149972473,0.6528,352.489568846 ),
( 1.39230801739,0.856123717386,0.6504,353.490959666 ),
( 1.4088202861,0.854385776453,0.648,354.492350487 ),
( 1.42536040635,0.852936680544,0.6456,355.493741307 ),
( 1.44192332582,0.851776872294,0.6432,356.495132128 ),
( 1.45850398525,0.850906705977,0.6408,357.496522949 ),
( 1.47509731996,0.85032644739,0.6384,358.497913769 ),
( 1.49169826138,0.850036273779,0.636,359.49930459 ),
( 1.50830173862,0.850036273779,0.6336,360.0 ),
( 1.5,0.85,0.6,360.0 ),

]

landing = [

( 1.5,0.85,0.6,360.0 ),
( 1.5,0.85,0.55,360.0 ),
( 1.5,0.85,0.5,360.0 ),
( 1.5,0.85,0.45,360.0 ),
( 1.5,0.85,0.4,360.0 ),
( 1.5,0.85,0.35,360.0 ),
( 1.5,0.85,0.3,360.0 ),
( 1.5,0.85,0.25,360.0 ),
( 1.5,0.85,0.2,360.0 ),
( 1.5,0.85,0.1,360.0 ),
( 1.5,0.85,0.0,360.0 ),

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
    
    cf.param.set_value('locSrv.extPosStdDev', 0.04)

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
    
    
def vel_callback(timestamp, data, logconf3):
    global vx_ekf
    global vy_ekf
    #global zv_ekf

    vx = data['stateEstimate.vx']
    vy = data['stateEstimate.vy']
    
    #print('vel: ( {}, {})'.format(vx, vy))

    
    
    vx_ekf = vx;
    vy_ekf = vy;
    #vz_ekf = vz;
    


def position_callback(timestamp, data, logconf):
    global x_ekf
    global y_ekf
    global z_ekf
    
    #global xv_ekf
    #global yv_ekf
    #global zv_ekf

    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    
    #vx = data['kalman.statePX']
    #vy = data['kalman.statePY']
    #vz = data['kalman.statePZ']
    #print('pos: ({}, {}, {})'.format(x, y, z))
    x_ekf = x;
    y_ekf = y;
    z_ekf = z;
    
    #vx_ekf = vx;
    #vy_ekf = vy;
    #vz_ekf = vz;
    


def start_position_printing(cf):
    log_conf = LogConfig(name='Position', period_in_ms=250)
    log_conf2 = LogConfig(name='Stabilizer', period_in_ms=250)
    log_conf3 = LogConfig(name='stateEstimate', period_in_ms=50)

    
    

    
    log_conf2.add_variable('stabilizer.yaw', 'float')
    
    log_conf3.add_variable('stateEstimate.vx', 'float')
    log_conf3.add_variable('stateEstimate.vy', 'float')



    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')


    cf.log.add_config(log_conf)
    cf.log.add_config(log_conf2)
    cf.log.add_config(log_conf3)

    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()
    
    log_conf2.data_received_cb.add_callback(yaw_callback)
    log_conf2.start()
    
    log_conf3.data_received_cb.add_callback(vel_callback)
    log_conf3.start()
    
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
    flight_time = 0.5;

    while (True):
        if capture:
            cap_num = cap_num+1;
            camera.capture('image/image' +str(cap_num)+'.jpg')
            #print("captured")
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
    
    global vision_yaw;
    global vision_yaw_last;
    
    vision_yaw=0;
    vision_yaw_last=0;
    global d1;
    global d2;
    global d3;
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
        if ret==True:
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

                        if col>=7 and col<14:
                            col = col+14
                        elif col>=21 and col<28:
                            col = col-14

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

                end_time = time.time()
                DT=end_time - start_time
                #print("t:  ", (int)(1000*(DT)))
                start_time = time.time()
                
     
                d1=float(d1/100)
                d2=float(d2/100)
                d3=float(1.0*(d3/100))
                #print (vision_yaw,yaw_setpoint, yaw_)
                #print ("t:  ", (int)(1000*(DT)),"  ", d1,d2)



            cf.extpos.send_extpos(float(d1),float(d2),float(d3))
            print (d1,d2)

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
        
        #camera = PiCamera()
        #camera.vflip = True
        #camera.hflip = True
        #camera.rotation = 270

        #camera.resolution = (640, 480)

        #print("Start RPi camera preview.")
        #time.sleep(3)
        #camera.start_recording('/home/pi/crazyflie/crazyflie-lib-python/image/video1.h264')
        #print("recording")

        
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

        for position in takeoff:
            for i in range(12):
                cf.commander.send_position_setpoint(position[1],position[0],position[2],0)
                time.sleep(0.03)



        x_pose_setpoint = 0.8;
        y_pose_setpoint = 1.5;
        error_threshold = 0.10

        for position in circle_trajectory_yaw4:
        #if 2>1:
            #print ("while out  ",position[0],position[1],d1, d2,y_pose_setpoint,x_pose_setpoint, abs(y_pose_setpoint - d1) ,abs(x_pose_setpoint - d2) )
            for i in range(10):
            #while ( abs(position[1] - d1) > error_threshold  or abs(position[0] - d2) > error_threshold  ):
                #print ("while  ")
                x_pose_setpoint = position[0];
                y_pose_setpoint = position[1];
                z_pose_setpoint = position[2];
                #yaw_setpoint    = position[3];
                #print(int(position[3]))
                if int(position[3]) == 0:
                    MOVE_FLAG=0;
                elif int(position[3]) ==360:
                    MOVE_FLAG=2;
                else:
                    MOVE_FLAG=1;

                #print (d1,d2,d3)
                
                loc_y = d2;
                loc_x = d1;
                #print (Cx,Cy)
                if MOVE_FLAG==1:
                    ###########################################
                    if (d1 > Cx and d2 > Cy ):
                        ZONE = 2;
                    elif(d1 <= Cx and d2 >= Cy ):
                        ZONE = 1;
                    elif((d1 < Cx and d2 <  Cy )):
                        ZONE = 4;
                    else:
                        ZONE = 3;
                    ###########################################

                    if ZONE ==1:
                            if (last_ZONE ==1 or last_ZONE==0 or last_ZONE ==2) :
                                last_ZONE = 1;

                                if loc_x == Cx:
                                    yaw_setpoint = -90
                                else:
                                    yaw_setpoint = - 57.2958*(math.atan((loc_y - Cy)/(Cx -loc_x)))
                            elif last_ZONE ==4:
                                last_ZONE = 4;

                                yaw_setpoint = -360 -57.2958*(math.atan((loc_y - Cy)/(Cx -loc_x)))


                    elif ZONE ==2:
                            
                            if loc_y ==Cy:
                                yaw_setpoint = -180
                            else:
                                yaw_setpoint =  -90 - 57.2958*math.atan((loc_x - Cx)/(loc_y - Cy))
                            last_ZONE = 2;
                    elif ZONE ==3:
                            
                            if loc_y ==Cy:
                                yaw_setpoint = -180
                            else:
                                yaw_setpoint = -360+90+ 57.2958*math.atan((loc_x - Cx) / (Cy - loc_y))
                            last_ZONE = 3;
                    else:
                            if (last_ZONE ==4 or last_ZONE ==3):
                                last_ZONE = 4;
                                if Cx == loc_x:
                                    yaw_setpoint = -360
                                else:
                                    yaw_setpoint = -360 + 57.2958*math.atan((Cy - loc_y)/(Cx - loc_x))
                            elif last_ZONE ==1 or last_ZONE ==0:
                                last_ZONE = 1;
                                yaw_setpoint = 57.2958*math.atan((Cy - loc_y)/(Cx - loc_x))

                            
                elif MOVE_FLAG==0:
                    yaw_setpoint = 0;
                elif MOVE_FLAG==2:
                    yaw_setpoint = -360.0;

                #print (yaw_setpoint,position[3],vision_yaw,yaw_,x_pose_setpoint,y_pose_setpoint,z_pose_setpoint,d1,d2,(yaw_setpoint -position[3])/2)
                #############################################################################
                #print (ZONE,yaw_setpoint,d1,d2)

                #yaw_setpoint = (yaw_setpoint -position[3])/2
                cf.commander.send_position_setpoint(position[1],position[0],position[2],yaw_setpoint)
                             
                time.sleep(0.015)
                
                #print (yaw_setpoint,position[3],vision_yaw,yaw_,x_pose_setpoint,y_pose_setpoint,z_pose_setpoint,d1,d2,y_ekf,x_ekf,z_ekf,vy_ekf,vx_ekf)

        for position in landing:
                for i in range(12):
                    cf.commander.send_position_setpoint(position[1], position[0], position[2], -360.0)
                    time.sleep(0.03)

        #cf.commander.send_stop_setpoint()
        print("stop")

        camera.stop_recording()


       