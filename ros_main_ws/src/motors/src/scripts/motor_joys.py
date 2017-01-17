#!/usr/bin/env python
#license removed for brevity

import os
import sys
import pygame

import rospy
import json
from motor_control import PerformMove, PerformSpin, switchOffMotors, JoystickEvent
from std_msgs.msg import String

driveLeft = 0.0
driveRight = 0.0

def callback(data):
    json_data2 = json.loads(data.data)
    print("Right")
    print (json_data2['right_speed'])
    print("Left")
    print (json_data2['left_speed'])
    runJoystick()
    
def runJoystick():
    
    # Handle each event individually
    JoystickEvent(driveLeft, driveRight)


if __name__ == '__main__':

    rospy.init_node('motor_listener', anonymous=True)
    pub = rospy.Publisher('motors_publisher', String, queue_size = 30)
    rate = rospy.Rate(10) # 10hz

    try:
        while not rospy.is_shutdown():
            right_speed = driveRight
            left_speed =  driveLeft
            json_data = { 
                          'right_speed' : right_speed,
                          'left_speed' : left_speed 
                        }
            pub.publish(json.dumps(json_data))
            rospy.loginfo(json.dumps(json_data))
            rospy.Subscriber("motors_publisher", String, callback)
            rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        switchOffMotors()
        pass
