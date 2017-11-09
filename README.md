# Accurate Robot Motion Control without External Reference System 

This is UTP Petrobot's first major step into autonomous mobile robotics. For V1.0, we focus our efforts into developing autonomous
control algorithm and system to control a robot’s movement without external reference (e.g. wall, line, etc.) in 2D space. 
Our control parameters include the robot's:
* DIRECTION
* TRAVEL DISTANCE
* COORDINATE IN SPACE

Note that this robot is not capable of sensing and responding to external stimuli. It is able to move in two ways: going straight for a
specified distance and turning on spot for a specified angle. All movements are pre-programmed parameters. 

### Area:
Two-wheeled drive

### Input Sensors:
* MPU 9250
* Rotary Encoder B106

### Research aspects
1. MPU 9250 sensor
    
    We choose the MPU 9250 because it serves as the robot’s absolute positioning reference. The MPU 9250 is a combination of 3 inertial sensors, the Accelerometer, Gyroscope & Magnetometer. It was able to give us the yaw, pitch and roll angle of the robot with its orientation based of the Earth’s magnetic heading. 
    



* [5 Product Create Form](www.google.com)
