#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data.data)
    print data.data
    
def listener():
    rospy.init_node('ping_sub', anonymous=True)
    rospy.Subscriber("ping", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
