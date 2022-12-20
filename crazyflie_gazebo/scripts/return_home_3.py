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

px1 = -0.186
py1 = -0.064

px2 = 0.535
py2 = 0.494

px3 = 1.061
py3 = -0.52

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf1 = crazyflie.Crazyflie("cf1", "/cf1")
    cf2 = crazyflie.Crazyflie("cf2", "/cf2")
    cf3 = crazyflie.Crazyflie("cf3", "/cf3")

    cf1.setParam("commander/enHighLevel", 1)
    cf2.setParam("commander/enHighLevel", 1)
    cf3.setParam("commander/enHighLevel", 1)

    cf1.takeoff(targetHeight = 0.5, duration = 2.0)
    cf2.takeoff(targetHeight = 1.0, duration = 2.0)
    cf3.takeoff(targetHeight = 1.5, duration = 2.0)
    time.sleep(5.0)

    cf1.goTo(goal = [px1, py1, 0.5], yaw=0.0, duration = 4.0, relative = False)
    cf2.goTo(goal = [px2, py2, 1.0], yaw=0.0, duration = 4.0, relative = False)
    cf3.goTo(goal = [px3, py3, 1.5], yaw=0.0, duration = 4.0, relative = False)
    time.sleep(5.0)

    cf1.land(targetHeight = 0.0, duration = 2.0)
    cf2.land(targetHeight = 0.0, duration = 2.0)
    cf3.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(5.0)

    cf1.stop()
    cf2.stop()
    cf3.stop()

