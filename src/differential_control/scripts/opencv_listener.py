#!/usr/bin/env python
'''
This script is for processing the image that we get from the camera.
The reason for separating image processing and image capture is faster image processing. 
Raspberry pi can handle this image processing but it will make the image capturing slower
In this script, we are trying to find a colored blob inside our image. We are using the color yellow as our default.
To achieve this process we do some image morphological techniques which are opening & closing.
Opening is for removing noises & closing is for unifying our small objects into one
''' 
import rospy
import cv2
import numpy as np
from scipy.interpolate import interp1d
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image

#We define the linear & angular range for our twist message
linear_range=[0,0.5]
angular_range=[-0.5,0.5]

class Image_Twist():

    def __init__(self):
        
        #Define our node which publishes the '/camera/cmd_vel' topic and subscribe to '/camera' topic
        self.publisher = rospy.Publisher('/camera/cmd_vel',Twist,queue_size=1)
        rospy.init_node('OpenCV', anonymous=False)
        self.subscriber=rospy.Subscriber('/Camera', Image, self.ImageCallback)
        
        #Define some parameters for our image processing
        self._lower_color = rospy.get_param('~lower_color',[10,150,150])
        self._upper_color = rospy.get_param('~upper_color',[30,240,240])
        self._kernel_close_size = rospy.get_param('~kernel_close_size',(3,3))
        self._kernel_close = np.ones(self._kernel_close_size,np.uint8)
        self._kernel_open_size = rospy.get_param('~kernel_open_size',(3,3))
        self._kernel_open = np.ones(self._kernel_open_size,np.uint8)
        self._pixel_size = rospy.get_param('~pixel_size',[240,240])
        self._area_range = rospy.get_param('~area_range',[0.05,0.1])
        self.min_area = self._area_range[0]*self._pixel_size[0]*self._pixel_size[1]
        self.max_area = self._area_range[1]*self._pixel_size[0]*self._pixel_size[1]
        
        #Define our interpolation boundary using interp1d from scipy    
        self.linear_interpolation = interp1d([self.max_area,self.min_area],linear_range,bounds_error=False,fill_value=(100,0))
        self.angular_interpolation = interp1d([0,self._pixel_size[0]],angular_range)
        
        #Define our message & CvBridge for receiving images 
        self.bridge = CvBridge()        
        self.cv_image = np.zeros((self._pixel_size[0],self._pixel_size[1]))
        self.twist_msg = Twist()
        
    def ImageCallback(self,data):
        
        #Change the Image message type from our '/camera' topic to cv2 image
        try:           
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except  CvBridgeError as (e):
            rospy.loginfo(e)
        

    def twist_publisher(self):
        
        #Publisher & image processing will only start when cv_image is present
        if (self.cv_image.any()):
            
            #Start processing our image into blobs
            self.hsv_image = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
            self.mask = cv2.inRange(self.hsv_image, self._lower_color, self._upper_color)
            self.color_object = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.mask)
            self.color_grayscale = cv2.cvtColor(self.color_object, cv2.COLOR_BGR2GRAY)            
            self.color_close = cv2.morphologyEx(self.color_grayscale, cv2.MORPH_OPEN, kernel1)
            self.color_open = cv2.morphologyEx(self.color_close, cv2.MORPH_CLOSE, kernel2)
            self.contours = cv2.findContours(self.color_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

            #From the identified blobs, we find the biggest blob which most likely correspond to our object of interest
            self.x_max,self.y_max,self.w_max,self.h_max = 0,0,0,0        
            for item in range(len(self.contours)):
                cnt = self.contours[item]
                x, y, w, h = cv2.boundingRect(cnt)
                if(self.w_max*self.h_max<w*h):
                    self.x_max,self.y_max,self.w_max,self.h_max = x,y,w,h

            #Find the center & area of our blob then make rectangle around the blob            
            self.area = self.w_max*self.h_max    
            self.center_x,self.center_y = self.x_max+(self.w_max/2),self.y_max+(self.h_max/2)            
            print(self.center_x,self.center_y,self.area)
            cv2.rectangle(self.cv_image, (self.x_max, self.y_max), (self.x_max + self.w_max, self.y_max + self.h_max), (0, 255, 0), 2)
            cv2.imshow("image",self.cv_image)
            cv2.waitKey(1)         

            #If we have a blob, we interpolate that area & center into twist message. If not, we send 0
            if (self.w_max!=0):
                self.twist_msg.angular.z = float(self.angular_interpolation(self.center_x))
                self.twist_msg.linear.x = float(self.linear_interpolation(self.area))
                self.publisher.publish(self.twist_msg)
                rospy.loginfo("Twist sent")
            else:
                self.twist_msg.angular.z = 0
                self.twist_msg.linear.x = 0
                self.publisher.publish(self.twist_msg)
                rospy.loginfo("No yellow object")      
    
    def run(self):        
        #Start our node & define the rate
        rospy.loginfo("Starting")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.twist_publisher()
            rate.sleep() 

if __name__ == '__main__':
    
    #Define our object & run it    
    raspi_camera_twist = Image_Twist()
    raspi_camera_twist.run()
