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
   starttime = time.time()

   while GPIO.input(pin)==0:
      starttime = time.time()

   while GPIO.input(pin)==1:
      endtime = time.time()
      
   duration = endtime - starttime
   # Distance is defined as time/2 (there and back) * speed of sound 34000 cm/s 
   distance = duration * 34000 / 2
   return distance

#Normalize distance
def Normalize(distance):
    normalized_value = 0; #too far
    if distance <= 20:
        #aux = distance / 20;
        #normalized_value = (-1) * (1 - aux);
        normalized_value = 1; #too close
    return normalized_value

# Publish value sensor
def ping_distance():

    rospy.init_node('ping_pub', anonymous=True)
    pub = rospy.Publisher('ping', String, queue_size=100)
    rate = rospy.Rate(5) # 10hz
    print("Ping running...")
    while not rospy.is_shutdown():
        distance = ReadDistance(11)
        #print distance
        normalize = Normalize(distance)
        pub.publish(str(normalize))
        rate.sleep()

if __name__ == '__main__':
    try:
        ping_distance()
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
