#!/usr/bin/env python
#license removed for brevity
import sys
import rospy
import json
from motor_control import *
from std_msgs.msg import String


voltageIn = 12.0
voltageOut = 6.0
global maxPower

if len(sys.argv) == 2:
    voltageIn = sys.argv[1]
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)

print('voltage in: %s' % (voltageIn))

def callback(data):
    speed = json.loads(data.data)
    #print speed
    speed_r = float(speed['right_speed'])
    speed_l = float(speed['left_speed'])

    if -1 <= speed_r <= 1 and -1 <= speed_l <=1:
        PerformMove(speed_l, speed_r, 0.1, maxPower)
        
if __name__ == '__main__':
    global velocity
    rospy.init_node('motor_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    try:
        while not rospy.is_shutdown():
            rospy.Subscriber("motors", String, callback)
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
