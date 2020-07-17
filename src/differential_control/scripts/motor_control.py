#!/usr/bin/env python
'''
This script is used for controlling the differential robot
I am using a motor driver (L295N) with pin configuration as below:
-12 & 16 for left motor
-11 & 13 for right motor
-32 & 33 for PWM control

For a motor driver, we control the speed using PWM & the other 4 pins will be used
to determine motor direction
'''

import rospy
from raspi_pwm.msg import pwm
import RPi.GPIO as GPIO

#Define our pins
GPIO.setwarnings(False)
GPIO.setmode (GPIO.BOARD)
channels = [32,33,12,16,11,13]
GPIO.setup(channels,GPIO.OUT)

#Define PWM pin & start with 0
p1 = GPIO.PWM(32,100)
p1.start(0)
p2 = GPIO.PWM(33,100)
p2.start(0)

#Define the callback for our subscriber
def pwm_info_callback(data):
    
    #update user
    print("Controlling with PWM value 1: %r" %data.pwm_value_1)
    print("Direction of motor 1: %s" %data.direction_1)
    print("Controlling with PWM value 2: %r" %data.pwm_value_2)
    print("Direction of motor 2: %s" %data.direction_2)    
    
    #Change our pin state based on the direction
    if(data.direction_1 == "Forward"):
        GPIO.output(12,GPIO.LOW)
        GPIO.output(16,GPIO.HIGH)
    else:
        GPIO.output(12,GPIO.HIGH)
        GPIO.output(16,GPIO.LOW)
    if(data.direction_2 == "Forward"):
        GPIO.output(11,GPIO.HIGH)

    else:
        GPIO.output(11,GPIO.LOW)
        GPIO.output(13,GPIO.HIGH)
    
    #Update our PWM cycle
    p1.ChangeDutyCycle(data.pwm_value_1)
    p2.ChangeDutyCycle(data.pwm_value_2)

def pwm_info_subscriber():
    
    #Start our node
    rospy.init_node('Motor_control',anonymous=False)
    rospy.Subscriber('/pwm_info',pwm,pwm_info_callback)
    rospy.spin()

if __name__ == '__main__':
    #Start our program
    rospy.loginfo('Starting')
    pwm_info_subscriber()
