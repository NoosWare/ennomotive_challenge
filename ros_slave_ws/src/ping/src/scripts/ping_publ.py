#!/usr/bin/env python
#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import rospy
import json
from std_msgs.msg import String

# Use board based pin numbering
GPIO.setmode(GPIO.BOARD)

# Read values from sensor
def ReadDistance(pin):
   GPIO.setup(pin, GPIO.OUT)
   GPIO.output(pin, 0)
   time.sleep(0.000002)

   #send trigger signal
   GPIO.output(pin, 1)
   time.sleep(0.000005)
   GPIO.output(pin, 0)
   GPIO.setup(pin, GPIO.IN)

   while GPIO.input(pin)==0:
      starttime = time.time()

   while GPIO.input(pin)==1:
      endtime = time.time()
      
   duration = endtime - starttime
   # Distance is defined as time/2 (there and back) * speed of sound 34000 cm/s 
   distance = duration * 34000 / 2
   return distance

# Publish value sensor
def ping_distance():

    rospy.init_node('ping_pub', anonymous=True)
    pub = rospy.Publisher('ping', String, queue_size=100)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        distance = ReadDistance(11)
        json_data = { 'distance' : distance }
        pub.publish(json.dumps(json_data))
        rate.sleep()

if __name__ == '__main__':
    try:
        ping_distance()
    except rospy.ROSInterruptException:
        pass
