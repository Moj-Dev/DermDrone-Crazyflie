# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/

"""
2 circles
"""
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

x_pose_setpoint=0;
y_pose_setpoint=0;
z_pose_setpoint=0;



x_ekf=0;
y_ekf=0;
z_ekf=0;




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

( 1.5,0.9,0.84,0 ),
( 1.5,0.9,0.88,0 ),
( 1.5,0.9,0.92,0 ),
( 1.5,0.9,0.96,0 ),
( 1.5,0.9,1.0,0 ),
( 1.5,0.9,1.04,0 ),
( 1.5,0.9,1.08,0 ),
( 1.5,0.9,1.12,0 ),
( 1.5,0.9,1.16,0 ),
( 1.5,0.9,1.2,0 ),
( 1.5,0.9,1.24,0 ),
( 1.5,0.9,1.28,0 ),
( 1.5,0.9,1.32,0 ),
( 1.5,0.9,1.36,0 ),
( 1.5,0.9,1.4,0 ),
( 1.5,0.9,1.44,0 ),
( 1.5,0.9,1.48,0 ),
( 1.5,0.9,1.52,0 ),
( 1.5,0.9,1.56,0 ),
( 1.5,0.9,1.6,0 ),
( 1.5,0.9,1.6,0 ),
( 1.5,0.9,1.6,0 ),
( 1.5,0.9,1.6,0 ),
( 1.5,0.9,1.6,0 ),
( 1.5,0.9,1.6,0 ),
( 1.5,0.9,1.6,0 ),

( 1.5,0.9,1.6,0.0 ),
( 1.53939414689,0.900862579363,1.6,2.50871080139 ),
( 1.57871278139,0.903448664021,1.6,5.01742160279 ),
( 1.61788053584,0.907753296856,1.6,7.52613240418 ),
( 1.65682233181,0.913768226564,1.6,10.0348432056 ),
( 1.69546352398,0.921481923468,1.6,12.543554007 ),
( 1.73373004326,0.930879601621,1.6,15.0522648084 ),
( 1.77154853874,0.941943247151,1.6,17.5609756098 ),
( 1.80884651829,0.954651652784,1.6,20.0696864111 ),
( 1.84555248755,0.968980458503,1.6,22.5783972125 ),
( 1.88159608693,0.984902198236,1.6,25.0871080139 ),
( 1.91690822651,1.00238635251,1.6,27.5958188153 ),
( 1.95142121847,1.02139940694,1.6,30.1045296167 ),
( 1.98506890681,1.04190491649,1.6,32.6132404181 ),
( 2.0177867942,1.06386357531,1.6,35.1219512195 ),
( 2.04951216558,1.08723329211,1.6,37.6306620209 ),
( 2.0801842084,1.1119692708,1.6,40.1393728223 ),
( 2.10974412916,1.1380240964,1.6,42.6480836237 ),
( 2.13813526614,1.16534782589,1.596,45.1567944251 ),
( 2.16530319799,1.19388808398,1.592,47.6655052265 ),
( 2.19119584804,1.22359016347,1.588,50.1742160279 ),
( 2.21576358415,1.25439713013,1.584,52.6829268293 ),
( 2.23895931382,1.28624993185,1.58,55.1916376307 ),
( 2.26073857448,1.31908751181,1.576,57.7003484321 ),
( 2.28105961871,1.35284692552,1.572,60.2090592334 ),
( 2.29988349426,1.38746346148,1.568,62.7177700348 ),
( 2.3171741187,1.42287076523,1.564,65.2264808362 ),
( 2.33289834864,1.45900096652,1.56,67.7351916376 ),
( 2.34702604318,1.49578480943,1.556,70.243902439 ),
( 2.35953012176,1.53315178511,1.552,72.7526132404 ),
( 2.37038661602,1.57103026693,1.548,75.2613240418 ),
( 2.37957471576,1.60934764779,1.544,77.7700348432 ),
( 2.38707680883,1.64803047928,1.54,80.2787456446 ),
( 2.39287851492,1.68700461248,1.536,82.787456446 ),
( 2.39696871304,1.72619534011,1.532,85.2961672474 ),
( 2.39933956295,1.76552753968,1.528,87.8048780488 ),
( 2.39998652008,1.80492581755,1.524,90.3135888502 ),
( 2.39890834432,1.8443146534,1.52,92.8222996516 ),
( 2.39610710238,1.88361854501,1.516,95.331010453 ),
( 2.39158816378,1.922762153,1.512,97.8397212544 ),
( 2.38536019063,1.96167044521,1.508,100.348432056 ),
( 2.37743512096,2.00026884056,1.504,102.857142857 ),
( 2.36782814591,2.03848335198,1.5,105.365853659 ),
( 2.35655768052,2.07624072824,1.496,107.87456446 ),
( 2.34364532852,2.11346859439,1.492,110.383275261 ),
( 2.32911584085,2.15009559046,1.488,112.891986063 ),
( 2.31299706826,2.18605150823,1.484,115.400696864 ),
( 2.29531990791,2.22126742586,1.48,117.909407666 ),
( 2.27611824414,2.25567583996,1.476,120.418118467 ),
( 2.25542888352,2.289210795,1.472,122.926829268 ),
( 2.23329148431,2.32180800974,1.468,125.43554007 ),
( 2.20974848044,2.35340500044,1.464,127.944250871 ),
( 2.18484500014,2.38394120062,1.46,130.452961672 ),
( 2.15862877948,2.41335807718,1.456,132.961672474 ),
( 2.13115007083,2.44159924259,1.452,135.470383275 ),
( 2.10246154657,2.46861056296,1.448,137.979094077 ),
( 2.07261819809,2.49434026184,1.444,140.487804878 ),
( 2.04167723042,2.51873901943,1.44,142.996515679 ),
( 2.00969795252,2.54176006714,1.436,145.505226481 ),
( 1.97674166365,2.56335927723,1.432,148.013937282 ),
( 1.94287153586,2.58349524742,1.428,150.522648084 ),
( 1.90815249284,2.6021293802,1.424,153.031358885 ),
( 1.87265108557,2.61922595688,1.42,155.540069686 ),
( 1.83643536466,2.63475220599,1.416,158.048780488 ),
( 1.79957474996,2.64867836616,1.412,160.557491289 ),
( 1.7621398975,2.66097774311,1.408,163.066202091 ),
( 1.724202564,2.67162676089,1.404,165.574912892 ),
( 1.68583546938,2.68060500698,1.4,168.083623693 ),
( 1.64711215734,2.68789527151,1.396,170.592334495 ),
( 1.60810685438,2.69348358017,1.392,173.101045296 ),
( 1.56889432755,2.69735922107,1.388,175.609756098 ),
( 1.52954974111,2.69951476519,1.384,178.118466899 ),
( 1.49014851246,2.69994608071,1.38,180.6271777 ),
( 1.45076616756,2.69865234087,1.376,183.135888502 ),
( 1.41147819619,2.69563602554,1.372,185.644599303 ),
( 1.37235990722,2.69090291655,1.368,188.153310105 ),
( 1.33348628428,2.68446208651,1.364,190.662020906 ),
( 1.29493184199,2.67632588149,1.36,193.170731707 ),
( 1.25677048317,2.66650989731,1.356,195.679442509 ),
( 1.21907535715,2.65503294968,1.352,198.18815331 ),
( 1.18191871957,2.6419170381,1.348,200.696864111 ),
( 1.14537179388,2.62718730371,1.344,203.205574913 ),
( 1.10950463479,2.61087198111,1.34,205.714285714 ),
( 1.07438599403,2.59300234421,1.336,208.222996516 ),
( 1.0400831885,2.57361264629,1.332,210.731707317 ),
( 1.0066619713,2.55274005436,1.328,213.240418118 ),
( 0.974186405669,2.53042457791,1.324,215.74912892 ),
( 0.942718742157,2.5067089922,1.32,218.257839721 ),
( 0.912319299338,2.48163875628,1.316,220.766550523 ),
( 0.883046348173,2.45526192587,1.312,223.275261324 ),
( 0.854956000315,2.42762906121,1.308,225.783972125 ),
( 0.828102100549,2.39879313015,1.304,228.292682927 ),
( 0.802536123588,2.36880940666,1.3,230.801393728 ),
( 0.778307075395,2.33773536482,1.296,233.31010453 ),
( 0.755461399253,2.30563056869,1.292,235.818815331 ),
( 0.734042886737,2.27255655814,1.288,238.327526132 ),
( 0.714092593773,2.23857673084,1.284,240.836236934 ),
( 0.695648761941,2.2037562208,1.28,243.344947735 ),
( 0.678746745171,2.16816177345,1.276,245.853658537 ),
( 0.663418941977,2.13186161778,1.272,248.362369338 ),
( 0.649694733352,2.09492533548,1.268,250.871080139 ),
( 0.637600426448,2.05742372761,1.264,253.379790941 ),
( 0.627159204154,2.01942867886,1.26,255.888501742 ),
( 0.618391080654,1.98101301978,1.256,258.397212544 ),
( 0.611312863064,1.94225038715,1.252,260.905923345 ),
( 0.605938119216,1.90321508286,1.248,263.414634146 ),
( 0.602277151651,1.86398193146,1.244,265.923344948 ),
( 0.600336977869,1.82462613673,1.24,268.432055749 ),
( 0.600121316878,1.78522313757,1.236,270.940766551 ),
( 0.601630582067,1.74584846334,1.232,273.449477352 ),
( 0.604861880411,1.7065775891,1.228,275.958188153 ),
( 0.609809018018,1.66748579096,1.224,278.466898955 ),
( 0.616462512003,1.62864800175,1.22,280.975609756 ),
( 0.624809608661,1.59013866743,1.216,283.484320557 ),
( 0.634834307919,1.55203160434,1.212,285.993031359 ),
( 0.646517394001,1.51439985774,1.208,288.50174216 ),
( 0.659836472267,1.47731556178,1.204,291.010452962 ),
( 0.674766012132,1.44084980124,1.2,293.519163763 ),
( 0.691277396014,1.40507247525,1.196,296.027874564 ),
( 0.70933897418,1.37005216334,1.192,298.536585366 ),
( 0.728916125421,1.33585599394,1.188,301.045296167 ),
( 0.749971323411,1.30254951574,1.184,303.554006969 ),
( 0.77246420864,1.27019657203,1.18,306.06271777 ),
( 0.796351665779,1.23885917833,1.176,308.571428571 ),
( 0.821587906321,1.2085974035,1.172,311.080139373 ),
( 0.848124556354,1.17946925461,1.168,313.588850174 ),
( 0.875910749287,1.15153056576,1.164,316.097560976 ),
( 0.904893223349,1.12483489102,1.16,318.606271777 ),
( 0.935016423686,1.09943340181,1.156,321.114982578 ),
( 0.966222608852,1.0753747888,1.152,323.62369338 ),
( 0.998451961489,1.05270516858,1.148,326.132404181 ),
( 1.03164270299,1.03146799524,1.144,328.641114983 ),
( 1.0657312119,1.01170397712,1.14,331.149825784 ),
( 1.10065214592,0.993450998735,1.136,333.658536585 ),
( 1.13633856709,0.976744048178,1.132,336.167247387 ),
( 1.17272207013,0.961615150052,1.128,338.675958188 ),
( 1.2097329136,0.948093304081,1.124,341.18466899 ),
( 1.2473001535,0.93620442952,1.12,343.693379791 ),
( 1.28535177935,0.925971315475,1.116,346.202090592 ),
( 1.32381485214,0.917413577221,1.112,348.710801394 ),
( 1.3626156442,0.910547618598,1.108,351.219512195 ),
( 1.40167978051,0.90538660057,1.104,353.728222997 ),
( 1.44093238124,0.901940416,1.1,356.236933798 ),
( 1.48029820533,0.900215670682,1.096,358.745644599 ),
( 1.51970179467,0.900215670682,1.092,361.254355401 ),
( 1.55906761876,0.901940416,1.088,363.763066202 ),
( 1.59832021949,0.90538660057,1.084,366.271777003 ),
( 1.6373843558,0.910547618598,1.08,368.780487805 ),
( 1.67618514786,0.917413577221,1.076,371.289198606 ),
( 1.71464822065,0.925971315475,1.072,373.797909408 ),
( 1.7526998465,0.93620442952,1.068,376.306620209 ),
( 1.7902670864,0.948093304081,1.064,378.81533101 ),
( 1.82727792987,0.961615150052,1.06,381.324041812 ),
( 1.86366143291,0.976744048178,1.056,383.832752613 ),
( 1.89934785408,0.993450998735,1.052,386.341463415 ),
( 1.9342687881,1.01170397712,1.048,388.850174216 ),
( 1.96835729701,1.03146799524,1.044,391.358885017 ),
( 2.00154803851,1.05270516858,1.04,393.867595819 ),
( 2.03377739115,1.0753747888,1.036,396.37630662 ),
( 2.06498357631,1.09943340181,1.032,398.885017422 ),
( 2.09510677665,1.12483489102,1.028,401.393728223 ),
( 2.12408925071,1.15153056576,1.024,403.902439024 ),
( 2.15187544365,1.17946925461,1.02,406.411149826 ),
( 2.17841209368,1.2085974035,1.016,408.919860627 ),
( 2.20364833422,1.23885917833,1.012,411.428571429 ),
( 2.22753579136,1.27019657203,1.008,413.93728223 ),
( 2.25002867659,1.30254951574,1.004,416.445993031 ),
( 2.27108387458,1.33585599394,1.0,418.954703833 ),
( 2.29066102582,1.37005216334,0.996,421.463414634 ),
( 2.30872260399,1.40507247525,0.992,423.972125436 ),
( 2.32523398787,1.44084980124,0.988,426.480836237 ),
( 2.34016352773,1.47731556178,0.984,428.989547038 ),
( 2.353482606,1.51439985774,0.98,431.49825784 ),
( 2.36516569208,1.55203160434,0.976,434.006968641 ),
( 2.37519039134,1.59013866743,0.972,436.515679443 ),
( 2.383537488,1.62864800175,0.968,439.024390244 ),
( 2.39019098198,1.66748579096,0.964,441.533101045 ),
( 2.39513811959,1.7065775891,0.96,444.041811847 ),
( 2.39836941793,1.74584846334,0.956,446.550522648 ),
( 2.39987868312,1.78522313757,0.952,449.059233449 ),
( 2.39966302213,1.82462613673,0.948,451.567944251 ),
( 2.39772284835,1.86398193146,0.944,454.076655052 ),
( 2.39406188078,1.90321508286,0.94,456.585365854 ),
( 2.38868713694,1.94225038715,0.936,459.094076655 ),
( 2.38160891935,1.98101301978,0.932,461.602787456 ),
( 2.37284079585,2.01942867886,0.928,464.111498258 ),
( 2.36239957355,2.05742372761,0.924,466.620209059 ),
( 2.35030526665,2.09492533548,0.92,469.128919861 ),
( 2.33658105802,2.13186161778,0.916,471.637630662 ),
( 2.32125325483,2.16816177345,0.912,474.146341463 ),
( 2.30435123806,2.2037562208,0.908,476.655052265 ),
( 2.28590740623,2.23857673084,0.904,479.163763066 ),
( 2.26595711326,2.27255655814,0.9,481.672473868 ),
( 2.24453860075,2.30563056869,0.896,484.181184669 ),
( 2.2216929246,2.33773536482,0.892,486.68989547 ),
( 2.19746387641,2.36880940666,0.888,489.198606272 ),
( 2.17189789945,2.39879313015,0.884,491.707317073 ),
( 2.14504399969,2.42762906121,0.88,494.216027875 ),
( 2.11695365183,2.45526192587,0.876,496.724738676 ),
( 2.08768070066,2.48163875628,0.872,499.233449477 ),
( 2.05728125784,2.5067089922,0.868,501.742160279 ),
( 2.02581359433,2.53042457791,0.864,504.25087108 ),
( 1.9933380287,2.55274005436,0.86,506.759581882 ),
( 1.9599168115,2.57361264629,0.856,509.268292683 ),
( 1.92561400597,2.59300234421,0.852,511.777003484 ),
( 1.89049536521,2.61087198111,0.848,514.285714286 ),
( 1.85462820612,2.62718730371,0.844,516.794425087 ),
( 1.81808128043,2.6419170381,0.84,519.303135889 ),
( 1.78092464285,2.65503294968,0.836,521.81184669 ),
( 1.74322951683,2.66650989731,0.832,524.320557491 ),
( 1.70506815801,2.67632588149,0.828,526.829268293 ),
( 1.66651371572,2.68446208651,0.824,529.337979094 ),
( 1.62764009278,2.69090291655,0.82,531.846689895 ),
( 1.58852180381,2.69563602554,0.816,534.355400697 ),
( 1.54923383244,2.69865234087,0.812,536.864111498 ),
( 1.50985148754,2.69994608071,0.808,539.3728223 ),
( 1.47045025889,2.69951476519,0.804,541.881533101 ),
( 1.43110567245,2.69735922107,0.8,544.390243902 ),
( 1.39189314562,2.69348358017,0.796,546.898954704 ),
( 1.35288784266,2.68789527151,0.792,549.407665505 ),
( 1.31416453062,2.68060500698,0.788,551.916376307 ),
( 1.275797436,2.67162676089,0.784,554.425087108 ),
( 1.2378601025,2.66097774311,0.78,556.933797909 ),
( 1.20042525004,2.64867836616,0.776,559.442508711 ),
( 1.16356463534,2.63475220599,0.772,561.951219512 ),
( 1.12734891443,2.61922595688,0.768,564.459930314 ),
( 1.09184750716,2.6021293802,0.764,566.968641115 ),
( 1.05712846414,2.58349524742,0.76,569.477351916 ),
( 1.02325833635,2.56335927723,0.756,571.986062718 ),
( 0.99030204748,2.54176006714,0.752,574.494773519 ),
( 0.958322769582,2.51873901943,0.748,577.003484321 ),
( 0.927381801906,2.49434026184,0.744,579.512195122 ),
( 0.897538453431,2.46861056296,0.74,582.020905923 ),
( 0.868849929171,2.44159924259,0.736,584.529616725 ),
( 0.841371220523,2.41335807718,0.732,587.038327526 ),
( 0.81515499986,2.38394120062,0.728,589.547038328 ),
( 0.79025151956,2.35340500044,0.724,592.055749129 ),
( 0.766708515686,2.32180800974,0.72,594.56445993 ),
( 0.744571116481,2.289210795,0.716,597.073170732 ),
( 0.723881755865,2.25567583996,0.712,599.581881533 ),
( 0.704680092094,2.22126742586,0.708,602.090592334 ),
( 0.687002931743,2.18605150823,0.704,604.599303136 ),
( 0.670884159154,2.15009559046,0.7,607.108013937 ),
( 0.656354671483,2.11346859439,0.696,609.616724739 ),
( 0.643442319479,2.07624072824,0.692,612.12543554 ),
( 0.632171854092,2.03848335198,0.688,614.634146341 ),
( 0.622564879036,2.00026884056,0.684,617.142857143 ),
( 0.614639809375,1.96167044521,0.68,619.651567944 ),
( 0.608411836221,1.922762153,0.676,622.160278746 ),
( 0.603892897623,1.88361854501,0.672,624.668989547 ),
( 0.601091655677,1.8443146534,0.668,627.177700348 ),
( 0.600013479922,1.80492581755,0.664,629.68641115 ),
( 0.600660437054,1.76552753968,0.66,632.195121951 ),
( 0.603031286957,1.72619534011,0.656,634.703832753 ),
( 0.607121485084,1.68700461248,0.652,637.212543554 ),
( 0.612923191166,1.64803047928,0.648,639.721254355 ),
( 0.620425284245,1.60934764779,0.644,642.229965157 ),
( 0.629613383985,1.57103026693,0.64,644.738675958 ),
( 0.640469878241,1.53315178511,0.636,647.24738676 ),
( 0.652973956818,1.49578480943,0.632,649.756097561 ),
( 0.667101651361,1.45900096652,0.628,652.264808362 ),
( 0.682825881296,1.42287076523,0.624,654.773519164 ),
( 0.700116505743,1.38746346148,0.62,657.282229965 ),
( 0.718940381289,1.35284692552,0.616,659.790940767 ),
( 0.73926142552,1.31908751181,0.612,662.299651568 ),
( 0.761040686183,1.28624993185,0.608,664.808362369 ),
( 0.784236415854,1.25439713013,0.604,667.317073171 ),
( 0.808804151961,1.22359016347,0.6,669.825783972 ),
( 0.834696802011,1.19388808398,0.596,672.334494774 ),
( 0.861864733856,1.16534782589,0.592,674.843205575 ),
( 0.890255870838,1.1380240964,0.588,677.351916376 ),
( 0.919815791602,1.1119692708,0.584,679.860627178 ),
( 0.950487834421,1.08723329211,0.58,682.369337979 ),
( 0.982213205803,1.06386357531,0.576,684.87804878 ),
( 1.01493109319,1.04190491649,0.572,687.386759582 ),
( 1.04857878153,1.02139940694,0.568,689.895470383 ),
( 1.08309177349,1.00238635251,0.564,692.404181185 ),
( 1.11840391307,0.984902198236,0.56,694.912891986 ),
( 1.15444751245,0.968980458503,0.556,697.421602787 ),
( 1.19115348171,0.954651652784,0.552,699.930313589 ),
( 1.22845146126,0.941943247151,0.548,702.43902439 ),
( 1.26626995674,0.930879601621,0.544,704.947735192 ),
( 1.30453647602,0.921481923468,0.54,707.456445993 ),
( 1.34317766819,0.913768226564,0.536,709.965156794 ),
( 1.38211946416,0.907753296856,0.532,712.473867596 ),
( 1.42128721861,0.903448664021,0.528,714.982578397 ),
( 1.46060585311,0.900862579363,0.524,717.491289199 ),
    
( 1.5,0.9,0.52,720.0 ),
( 1.5,0.9,0.59,720.0 ),
( 1.5,0.9,0.5,720.0 ),
( 1.5,0.9,0.45,720.0 ),
( 1.5,0.9,0.4,720.0 ),
( 1.5,0.9,0.35,720.0 ),
( 1.5,0.9,0.3,720.0 ),
( 1.5,0.9,0.25,720.0 ),
( 1.5,0.9,0.2,720.0 ),
( 1.5,0.9,0.15,720.0 ),
( 1.5,0.9,0.10,720.0 ),
( 1.5,0.9,0.0,720.0 ),






    
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

    #cf.param.set_value('locSrv.extQuatStdDev', 0.06)


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
    log_conf = LogConfig(name='Position', period_in_ms=300)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()
    
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
    end_time=0
    start_time=0
    
    first_run_flag=True;


    cutoff_freq = 0.2;

    RC = 1.0/(2*3.14*cutoff_freq)
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
            
            last_x = d1;
            last_y = d2;
            last_z = d3;
            
            #print("x: ",x_point, "   y: ",y_point ,"z: ",z_point, "   yaw:  ",ypr[0][2]  )       
            #f.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % ((int)(d1), (int)(d2), (int)(d3), (int)(ypr[0][2]),(int)(100*y_pose_setpoint),(int)(100*x_pose_setpoint),(int)(100*z_pose_setpoint),(int)(100*y_ekf),(int)(100*x_ekf),(int)(100*z_ekf) ))

            #f.write( ''+ (str)d1 + ',' + (str)(d2) + ',' + (str)(d3) + ', 1 , 1 , 1 , 1 , 1 , 1' + '\n' )
         
            end_time = time.time()
            DT=end_time - start_time
            #print("t:  ", (int)(1000*(DT)))
            start_time = time.time()
            

            
            #print("x: ",x_point, "   y: ",y_point ,"z: ",z_point, "   yaw:  ",ypr[0][2]  )
            #if float(1.15*(z_point/200)) > 40: 
            d1=float(d1/100)
            d2=float(d2/100)
            d3=float(1.0*(d3/100))
            
            #print("x: ",d1, "   y: ",d2 ,"z: ",d3 , "Yaw", ypr[0][2]   )
            print d1,d2,d3, ypr[0][2],y_pose_setpoint,x_pose_setpoint,z_pose_setpoint,y_ekf,x_ekf,z_ekf  



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
        #else:
        #    print("no external pose")


        # display the resulting frame
        #cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    

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
        #t2 = threading.Thread(target=m_run_sequence, args=(scf,circle_trajectory_yaw4,))

        
        #t2 = threading.Thread(target=m_run_sequence, args=(scf,circle_trajectory_yaw4,))
        #t2.setDaemon(True)
        #t3.setDaemon(True)

        
        t1.start()
        t3.start()
        
        time.sleep(2)

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
            #cap_num = cap_num+1;
            #if cap_num%3 == 0:
            #    camera.capture('image/image' +str(cap_num)+'.jpg')
            #print('Setting position {}'.format(position))
            for i in range(110):
                #print("Sending setpoint")
                #print(i)
                x_pose_setpoint = position[0];
                y_pose_setpoint = position[1];
                z_pose_setpoint = position[2];
                
                cf.commander.send_position_setpoint(position[1],position[0],position[2],-position[3])
                
                
                time.sleep(0.004)
                #print("Sent setpoint")
                
        cf.commander.send_stop_setpoint()


        #while True:

        #    pass
        






        # We take off when the commander is created
        #with MotionCommander(scf) as mc:

 
