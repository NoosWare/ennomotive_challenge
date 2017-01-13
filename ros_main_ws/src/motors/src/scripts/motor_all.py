#!/usr/bin/env python
#license removed for brevity

import rospy
import json
from motor_control import PerformMove, PerformSpin
from std_msgs.msg import String

previous_velocity = 0
velocity = 0

def callback(data):
    json_data2 = json.loads(data.data)
    print(json_data2['right_speed'])
    #print(json_data2['left_speed'])
    changeValues()
    
def changeValues():
    global velocity, previous_velocity
    previous_velocity = velocity
    velocity = velocity + 0.1
    if velocity > 3:
        PerformMove(0, 0, 0.1)
    else:
        PerformMove(velocity, velocity, 0.1)
    #PerformSpin(0)

        
if __name__ == '__main__':

    global velocity
    rospy.init_node('motor_listener', anonymous=True)
    pub = rospy.Publisher('motors_publisher', String, queue_size = 30)
    rate = rospy.Rate(10) # 10hz

    try:
        while not rospy.is_shutdown():
            right_speed =  velocity
            left_speed =  velocity
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
