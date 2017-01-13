#!/usr/bin/env python
# coding: Latin-1
 
# Simple example of a motor sequence script
 
# Import library functions we need
import PicoBorgRev
import time
import math
import sys
 
# Setup the PicoBorg Reverse
PBR = PicoBorgRev.PicoBorgRev()
#PBR.i2cAddress = 0x44                  # Uncomment and change the value if you have changed the board address
PBR.Init()
if not PBR.foundChip:
    boards = PicoBorgRev.ScanForPicoBorgReverse()
    if len(boards) == 0:
        print 'No PicoBorg Reverse found, check you are attached :)'
    else:
        print 'No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
        print 'PBR.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#PBR.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
PBR.SetCommsFailsafe(False)             # Disable the communications failsafe
PBR.ResetEpo()
 
# Movement settings (worked out from our DiddyBorg on a smooth surface)
timeForward1m = 5.7                     # Number of seconds needed to move about 1 meter
timeSpin360   = 4.8                     # Number of seconds needed to make a full left / right spin
testMode = False                        # True to run the motion tests, False to run the normal sequence
 
# Power settings
voltageIn = 12.0                        # Total battery voltage to the PicoBorg Reverse
voltageOut = 6.0                        # Maximum motor voltage
 
# Setup the power limits
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)
 
# Function to perform a general movement
def PerformMove(driveLeft, driveRight, numSeconds):
    # Set the motors running
    PBR.SetMotor1(driveRight * maxPower)
    PBR.SetMotor2(-driveLeft * maxPower)
    # Wait for the time
    time.sleep(numSeconds)
    # Turn the motors off
    PBR.MotorsOff()
 
# Function to spin an angle in degrees
def PerformSpin(angle):
    if angle < 0.0:
        # Left turn
        driveLeft  = -1.0
        driveRight = +1.0
        angle *= -1
    else:
        # Right turn
        driveLeft  = +1.0
        driveRight = -1.0
    # Calculate the required time delay
    numSeconds = (angle / 360.0) * timeSpin360
    # Perform the motion
    PerformMove(driveLeft, driveRight, numSeconds)
 
# Function to drive a distance in meters
def PerformDrive(meters):
    if meters < 0.0:
        # Reverse drive
        driveLeft  = -1.0
        driveRight = -1.0
        meters *= -1
    else:
        # Forward drive
        driveLeft  = +1.0
        driveRight = +1.0
    # Calculate the required time delay
    numSeconds = meters * timeForward1m
    # Perform the motion
    PerformMove(driveLeft, driveRight, numSeconds)
 
# Run test mode if required
if testMode:
    # Show settings
    print 'Current settings are:'
    print '    timeForward1m = %f' % (timeForward1m)
    print '    timeSpin360 = %f' % (timeSpin360)
    # Check distance
    raw_input('Check distance, Press ENTER to start')
    print 'Drive forward 30cm'
    PerformDrive(+0.3)
    raw_input('Press ENTER to continue')
    print 'Drive reverse 30cm'
    PerformDrive(-0.3)
    # Check spinning
    raw_input('Check spinning, Press ENTER to continue')
    print 'Spinning left'
    PerformSpin(-360)
    raw_input('Press ENTER to continue')
    print 'Spinning Right'
    PerformSpin(+360)
    print 'Update the settings as needed, then test again or disable test mode'
    sys.exit(0)
