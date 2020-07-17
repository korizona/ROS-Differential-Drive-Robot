#!/usr/bin/env python

import rospy
from raspi_pwm.msg import pwm
from geometry_msgs.msg import Twist
from numpy import interp
import time

class pwm_converter():
    
    def __init__(self):    
        self.publisher = rospy.Publisher('/pwm_info',pwm,queue_size=1)
        rospy.init_node('PWM_transform',anonymous=False)
        self.subscriber = rospy.Subscriber('/cmd_vel',Twist,self.twist_info_callback)
        self.subscriber = rospy.Subscriber('/camera/cmd_info',Twist,self.twist_info_callback)
        self._timeout = rospy.get_param('~timeout',2)
        self._wheelbase = rospy.get_param('~wheelbase',0.15)
        self._default_speed = rospy.get_param('~default_speed',1)
        self.pwm_msg = pwm()

    def twist_info_callback(self,data):
        rospy.loginfo("Get data & processing")
        self.pwm_msg.direction_1="Forward"    
        self.pwm_msg.direction_2="Forward"
        self.motor1 = (data.linear.x+data.angular.z)/2
        self.motor2 = (data.linear.x-data.angular.z)/2
        self.pwm_msg.pwm_value_1 = min(max(self.motor1,-100),100)
        self.pwm_msg.pwm_value_2 = min(max(self.motor2,-100),100)
        if (self.pwm_msg.pwm_value_1<0):
            self.pwm_msg.pwm_value_1=abs(self.pwm_msg.pwm_value_1)
            self.pwm_msg.direction_1="Backward"
        if(self.pwm_msg.pwm_value_2<0):
            self.pwm_msg.pwm_value_2=abs(self.pwm_msg.pwm_value_2)
            self.pwm_msg.direction_2="Backward"
        self.last_update = time.time()

    def pwm_info_publish(self):
        print("Controlling with PWM value 1: ",self.pwm_msg.pwm_value_1)
        print("Direction of motor 1: ",self.pwm_msg.direction_1)
        print("Controlling with PWM value 2: ",self.pwm_msg.pwm_value_2)
        print("Direction of motor 2: ",self.pwm_msg.direction_2)
        self.publisher.publish(self.pwm_msg)
            
    def run(self):
        rospy.loginfo("Starting")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pwm_info_publish()
            rate.sleep()

if __name__== "__main__":
    raspi_pwm_convert = pwm_converter()
    raspi_pwm_convert.run()

        
                    
        
