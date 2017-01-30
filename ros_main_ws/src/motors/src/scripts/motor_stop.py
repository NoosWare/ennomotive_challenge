#!/usr/bin/env python
#license removed for brevity

import os
import sys
import pygame

import rospy
from motor_control import PerformMove, PerformSpin, switchOffMotors
from std_msgs.msg import String

def stopMotors():
    
    # Handle each event individually
    PerformMove(0, 0, 1)
    switchOffMotors()

if __name__ == '__main__':

    rospy.init_node('motor_listener', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    start = rospy.Time.now().to_sec()

    try:
        while not rospy.is_shutdown():

            stopMotors()
            rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        switchOffMotors()
        pass
