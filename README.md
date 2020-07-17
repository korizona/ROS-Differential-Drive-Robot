# ROS-Differential-Drive-Robot
This is my first project after learning ROS for the first time. I am using a 3D printed chassis to make a differential drive robot. The robot has 4 wheels that can move either forward or backward only. The front and back wheels of a side (Left & Right) are controlled simulatenously from the motor driver. I am using L298N DC motor driver for controlling these motors. Raspberry pi model 3Bv2 is mounted on the chassis as the main controller for the robot.

This repository contains source code for simple differential drive control with ROS. There are 2 basic control scheme that are included. First, a keyboard controller to move the robot. Second, use a web camera to get images and approach the desired object (colored object). These two control scheme could be controlled by raspberry pi directly, however all the computing processes might make the Raspberry pi overheat. For that reason, we use a different machine to process the twist message from keyboard. In my case, i am using a laptop.

For connecting ROS from two machines, refer to this link http://wiki.ros.org/ROS/Tutorials/MultipleMachines

Future plans:
-Add an encoder for closed loop control process
-Add some distance sensor to avoid obstacles
