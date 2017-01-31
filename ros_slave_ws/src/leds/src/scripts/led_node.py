import time
import RPi.GPIO as GPIO
import rospy
import json
from std_msgs.msg import String


# Use board based pin numbering
GPIO.setmode(GPIO.BOARD)

def callback(data):
    led_data = json.loads(data.data)
    light = float(led_data["led"])
    TurnLed(light);

def TurnLed(light):
   pin = 11;
   #GPIO.setmode(GPIO.BCM)
   GPIO.setwarnings(False)
   GPIO.setup(pin,GPIO.OUT)
   if light == 1:
       GPIO.output(pin,GPIO.HIGH)
   else
       GPIO.output(pin,GPIO.LOW)
   
def receive_data():
    rospy.init_node('led', anonymous=True)

    while not rospy.is_shutdown():
        rospy.Subscriber("leds", String, callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        receive_data()
    except KeyboardInterrupt:
        TurnLed(0);    
    except rospy.ROSInterruptException:
        TurnLed(0);
