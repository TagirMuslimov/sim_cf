#!/usr/bin/env python

import imp
import rospy
import tf
import crazyflie
import crazymath_const_center
import time
import uav_trajectory
import math

from crazyflie_driver.msg import GenericLogData
from crazyflie_driver.msg import Position

timestr = time.strftime("%Y%m%d-%H%M%S")
fp = open('sim3_' + timestr + '_.csv', 'w')

CX = 0.0
CY = 0.0
k = 5.0
R = 0.65
v_f = 0.5
D_12 = 3*math.pi / 2
D_23 = 3*math.pi / 2

v_cruise = 0.5
k_f = 1

T_Z = 0.3

safety_radius = 0.3
eta_vf = 2.0
eta_apf = 0.2

def init_log(**log_vars):
    for k, v in log_vars.items():
        fp.write(str(k) + ';')
    fp.write('\n')


def write_log(**log_vars):
    for k, v in log_vars.items():
        fp.write(str(v) + ';')
    fp.write('\n')

# local position callback [if need]
def local_position_callback1(msg):
    global current_position1
    current_position1.x = msg.values[0]
    current_position1.y = msg.values[1]
    current_position1.z = msg.values[2]
    current_position1.header = msg.header
    
# local position callback [if need]
def local_position_callback2(msg):
    global current_position2
    current_position2.x = msg.values[0]
    current_position2.y = msg.values[1]
    current_position2.z = msg.values[2]
    current_position2.header = msg.header

# local position callback [if need]
def local_position_callback3(msg):
    global current_position3
    current_position3.x = msg.values[0]
    current_position3.y = msg.values[1]
    current_position3.z = msg.values[2]
    current_position3.header = msg.header


if __name__ == '__main__':
    rospy.init_node('test_high_level')

    # SUbscribe to get the local position of the crazyflie with prefix cf_prefix 
    current_position1 = Position()
    rospy.Subscriber("/cf1" + "/local_position" , GenericLogData , local_position_callback1)
    current_position2 = Position()
    rospy.Subscriber("/cf2" + "/local_position" , GenericLogData , local_position_callback2)
    current_position3 = Position()
    rospy.Subscriber("/cf3" + "/local_position" , GenericLogData , local_position_callback3)
    

    cf1 = crazyflie.Crazyflie("cf1", "/cf1")
    cf2 = crazyflie.Crazyflie("cf2", "/cf2")
    cf3 = crazyflie.Crazyflie("cf3", "/cf3")

    cf1.setParam("commander/enHighLevel", 1)
    cf2.setParam("commander/enHighLevel", 1)
    cf3.setParam("commander/enHighLevel", 1)

    cf1.takeoff(targetHeight = T_Z, duration = 2.0)
    cf2.takeoff(targetHeight = T_Z, duration = 2.0)
    cf3.takeoff(targetHeight = T_Z, duration = 2.0)
    time.sleep(5.0)

    steps = 2000
    paramCalc = crazymath_const_center.Crazymath3wAPF(CX, CY, v_cruise, v_f, k_f, D_12, D_23, k, R, safety_radius, eta_vf, eta_apf)
    for i in range(steps):
        cf1_params, cf2_params, cf3_params, p_12, p_23 = paramCalc.calculate(current_position1.x, current_position1.y, current_position2.x, current_position2.y, current_position3.x, current_position3.y)
        
        
        setPx1 = current_position1.x + cf1_params.vx/10
        setPx2 = current_position2.x + cf2_params.vx/10
        setPx3 = current_position3.x + cf3_params.vx/10

        setPy1 = current_position1.y + cf1_params.vy/10
        setPy2 = current_position2.y + cf2_params.vy/10
        setPy3 = current_position3.y + cf3_params.vy/10

        if i == 0:
            init_log(i=i, T_Z=T_Z, CX=CX, CY=CY, k=k, R=R, D_12=D_12, D_23=D_23,
                     v_f=v_f, v_cruise=v_cruise, k_f=k_f, p_12=p_12, p_23=p_23,
                     px_1=current_position1.x, py_1=current_position1.y, pz_1=current_position1.z, d_1=cf1_params.d, phi_1=cf1_params.phi, angle_1=cf1_params.angle, v1=cf1_params.v, vx1=cf1_params.vx, vy1=cf1_params.vy, setPx1=setPx1, setPy1=setPy1,
                     px_2=current_position2.x, py_2=current_position2.y, pz_2=current_position2.z, d_2=cf2_params.d, phi_2=cf2_params.phi, angle_2=cf2_params.angle, v2=cf2_params.v, vx2=cf2_params.vx, vy2=cf2_params.vy, setPx2=setPx2, setPy2=setPy2,
                     px_3=current_position3.x, py_3=current_position3.y, pz_3=current_position3.z, d_3=cf3_params.d, phi_3=cf3_params.phi, angle_3=cf3_params.angle, v3=cf3_params.v, vx3=cf3_params.vx, vy3=cf3_params.vy, setPx3=setPx3, setPy3=setPy3,
                     )

        write_log(i=i, T_Z=T_Z, CX=CX, CY=CY, k=k, R=R, D_12=D_12, D_23=D_23,
                     v_f=v_f, v_cruise=v_cruise, k_f=k_f, p_12=p_12, p_23=p_23,
                     px_1=current_position1.x, py_1=current_position1.y, pz_1=current_position1.z, d_1=cf1_params.d, phi_1=cf1_params.phi, angle_1=cf1_params.angle, v1=cf1_params.v, vx1=cf1_params.vx, vy1=cf1_params.vy, setPx1=setPx1, setPy1=setPy1,
                     px_2=current_position2.x, py_2=current_position2.y, pz_2=current_position2.z, d_2=cf2_params.d, phi_2=cf2_params.phi, angle_2=cf2_params.angle, v2=cf2_params.v, vx2=cf2_params.vx, vy2=cf2_params.vy, setPx2=setPx2, setPy2=setPy2,
                     px_3=current_position3.x, py_3=current_position3.y, pz_3=current_position3.z, d_3=cf3_params.d, phi_3=cf3_params.phi, angle_3=cf3_params.angle, v3=cf3_params.v, vx3=cf3_params.vx, vy3=cf3_params.vy, setPx3=setPx3, setPy3=setPy3,
                     )
        
        cf1.goTo(goal = [setPx1, setPy1, T_Z], yaw=0.0, duration = 0.1, relative = False)
        cf2.goTo(goal = [setPx2, setPy2, T_Z], yaw=0.0, duration = 0.1, relative = False)
        cf3.goTo(goal = [setPx3, setPy3, T_Z], yaw=0.0, duration = 0.1, relative = False)

        time.sleep(0.1)

    cf1.land(targetHeight = 0.0, duration = 2.0)
    cf2.land(targetHeight = 0.0, duration = 2.0)
    cf3.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(3.0)

    cf1.stop()
    cf2.stop()
    cf3.stop()

    fp.close()
