#!/usr/bin/env python
#license removed for brevity

import rospy
import json
from motor_control import PerformMove, PerformSpin
from std_msgs.msg import String

def callback(data):
    speed = json.loads(data.data)
    #print(speed['right_speed'])
    #print(speed['left_speed'])
    if float(speed['left_speed']) != 100 || float(speed['right_speed']) != 100:
        PerformMove(float(speed['left_speed']), float(speed['right_speed']), 0.1)
    else
        switchOffMotors()
        
        
if __name__ == '__main__':

    global velocity
    rospy.init_node('motor_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    try:
        while not rospy.is_shutdown():
            rospy.Subscriber("motors_controller", String, callback)
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
