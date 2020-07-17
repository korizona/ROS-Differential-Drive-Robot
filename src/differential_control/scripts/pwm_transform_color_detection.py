#!/usr/bin/env python
'''
This package is meant to be used with the teleop packages (teleop_twist_keyboard, etc.) as well as opencv_listener to get the twist
message from both of these nodes

For this package, we are using an open loop controller. Therefore, the control scheme is not perfect and needs
to be calibrated to suit the user's needs. The open loop controller equation is explained below
motor_speed = linear_speed + (angular_speed*wheelbase*angular_multiplier)/2
pwm_value = 100*motor_speed/(motor_speed_when_PWM_value_is_100)

-Linear_speed (m/s) & angular_speed (rad/s) are obtained from the twist message
-wheelbase (m) is the distance between the two wheels
-angular_multiplier is optional & it is used to ensure that the robot moves accordingly
-motor_speed_when_PWM_value_is_100 can be obtained from experiment or the motor's max speed. make sure to match this according to your motor specs

We combine our camera speed & teleop speed before publishing it to our motor_control 
'''
import rospy
from raspi_pwm.msg import pwm
from geometry_msgs.msg import Twist
import time

class pwm_converter():
    
    #initialize node & required parameters
    def __init__(self):
        
        #Define the node to subscribe to the topic '/cmd_vel' and publish to the topic '/pwm_value'    
        self.publisher = rospy.Publisher('/pwm_value',pwm,queue_size=1)
        rospy.init_node('PWM_transform',anonymous=False)
        self.subscriber = rospy.Subscriber('/cmd_vel',Twist,self.twist_info_callback)
        self.subscriber_image = rospy.Subscriber('/camera/cmd_vel',Twist,self.camera_twist_info_callback)
        
        #Define all parameters for our robot control
        self._timeout = rospy.get_param('~timeout',2)
        self._wheelbase = rospy.get_param('~wheelbase',0.15)
        self._default_speed = rospy.get_param('~default_speed',1)
        self._angular_multiplier = rospy.get_param('~angular_multiplier',2)
        
        #Define the message that will be published
        self.pwm_msg = pwm()
        
    def twist_info_callback(self,data):
        
        #Update the user & check current time for last update
        self.last_update = time.time()        
        rospy.loginfo("Get teleop data & processing")
        
        #Equation for controlling the motor        
        self.motor_teleop_1 = 100*(data.linear.x+(data.angular.z*self._wheelbase*self._angular_multiplier)/2)/self._default_speed
        self.motor_teleop_2 = 100*(data.linear.x-(data.angular.z*self._wheelbase*self._angular_multiplier)/2)/self._default_speed

     def camera_twist_info_callback(self,data):
        
        #Update the user & check current time for last update
        self.last_update = time.time()        
        rospy.loginfo("Get camera data & processing")
        
        #Equation for controlling the motor        
        self.motor_camera_1 = 100*(data.linear.x+(data.angular.z*self._wheelbase*self._angular_multiplier)/2)/self._default_speed
        self.motor_camera_2 = 100*(data.linear.x-(data.angular.z*self._wheelbase*self._angular_multiplier)/2)/self._default_speed
        

    def pwm_info_publish(self):
        
        #Combine the two speed
        self.pwm_msg.pwm_value_1 = self.motor_teleop_1 + self.motor_camera_1
        self.pwm_msg.pwm_value_2 = self.motor_teleop_2 + self.motor_camera_2  
        
        #Define the maximum & minimum value for PWM        
        self.pwm_msg.pwm_value_1 = min(max(self.pwm_msg.pwm_value_1,-100),100)
        self.pwm_msg.pwm_value_2 = min(max(self.pwm_msg.pwm_value_2,-100),100)      
        
        #Change the direction of motor if the PWM value is negative
        if (self.pwm_msg.pwm_value_1<0):
            self.pwm_msg.pwm_value_1=abs(self.pwm_msg.pwm_value_1)
            self.pwm_msg.direction_1="Backward"		 
        if(self.pwm_msg.pwm_value_2<0):
            self.pwm_msg.pwm_value_2=abs(self.pwm_msg.pwm_value_2)
            self.pwm_msg.direction_2="Backward"
        
        #Check the delay time. if it's longer than timeout, the motor needs to stop moving to prevent unwanted collision        
        self.delay_time = time.time()-self.last_update        
        if(self.delay_time < self._timeout):
            self.pwm_msg.pwm_value1=0
            self.pwm_msg.pwm_value2=0 
     
        #Publish message & update the user        
        rospy.loginfo("Controlling with PWM value 1: %r" %self.pwm_msg.pwm_value_1)
        rospy.loginfo("Direction of motor 1: %s" %self.pwm_msg.direction_1)
        rospy.loginfo("Controlling with PWM value 2: %r" %self.pwm_msg.pwm_value_2)
        rospy.loginfo("Direction of motor 2: %s" %self.pwm_msg.direction_2)
        self.publisher.publish(self.pwm_msg)
            
    def run(self):
        
        #Publish message with frequency 10 Hz
        rospy.loginfo("Starting")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pwm_info_publish()
            rate.sleep()

if __name__== "__main__":
    
    #Define object & run it    
    raspi_pwm_convert = pwm_converter()
    raspi_pwm_convert.run()

        
                    
        
