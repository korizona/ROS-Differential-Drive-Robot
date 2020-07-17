#!/usr/bin/env python
'''
This script is used for publishing Image message to our computer.
I am using a webcam in this project and not using raspberry pi camera.
To get our image, we are using the openCV package
'''

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

def ImagePublisher(cap):
    #Make new node
    publisher = rospy.Publisher('Camera',Image,queue_size = 10)
    rospy.init_node('Camera',anonymous=False)
    
    #Define rate of message published    
    rate = rospy.Rate(10)
    
    # Insert data for publishing
    bridge = CvBridge()
    
    #Get our image, transform it into Image message, and then publish it
    while not rospy.is_shutdown():              
        _,frame = cap.read()       
        frame = bridge.cv2_to_imgmsg(frame,"bgr8")
        rospy.loginfo('Sending')
        publisher.publish(frame)

if __name__ == '__main__':
    
    #Start our webcam stream
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 10)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    #Start our node
    try:
        ImagePublisher(cap)
    except rospy.ROSInterruptException:
        pass
