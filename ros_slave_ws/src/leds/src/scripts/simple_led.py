import time
import RPi.GPIO as GPIO

# Use board based pin numbering
GPIO.setmode(GPIO.BOARD)

def TurnLed(light):
   pin = 11;
   #GPIO.setmode(GPIO.BCM)
   GPIO.setwarnings(False)
   GPIO.setup(pin,GPIO.OUT)
   if light == 1:
       GPIO.output(pin,GPIO.HIGH)
   else
       GPIO.output(pin,GPIO.LOW)

if __name__ == '__main__':
    try:
        TurnLed(1)
    except KeyboardInterrupt:
        TurnLed(0)    
    except rospy.ROSInterruptException:
        TurnLed(0)
