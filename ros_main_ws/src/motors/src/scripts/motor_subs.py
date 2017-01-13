#!/usr/bin/env python
#license removed for brevity

import rospy
import json
from motor_control import PerformMove, PerformSpin
from std_msgs.msg import String

previous_velocity = 0

def callback(data):
    json_data = json.loads(data.data)
    print(json_data['right_speed'])
    print(json_data['left_speed'])
    if float(json_data['right_speed']) == 0:
        changeValues()
    
def changeValues():

    PerformMove(1, 1, 0.1)
    PerformSpin(0)

def listener():

    rospy.init_node('motor_listener', anonymous=True)
    
    while not rospy.is_shutdown():
        rospy.Subscriber("motors_publisher", String, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    listener()

