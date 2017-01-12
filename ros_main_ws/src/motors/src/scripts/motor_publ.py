#!/usr/bin/env python
#license removed for brevity
import rospy
import json
from motor_control import GetRightSpeed, GetLeftSpeed 
from std_msgs.msg import String

def speed_values():
    pub = rospy.Publisher('motors_publisher', String, queue_size = 20)
    rospy.init_node('motors_value', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        right_speed = GetRightSpeed()
        left_speed = GetLeftSpeed()
        json_data = { 
                      'rigth_speed' : right_speed,
                      'left_speed' : left_speed 
                    }
        pub.publish(json.dumps(json_data))
        rate.sleep()

if __name__ == '__main__':
    try:
        speed_values()
    except rospy.ROSInterruptException:
        pass
