# Accurate Robot Motion Control without External Reference System 

This is UTP Petrobot's first major step into autonomous mobile robotics. For V1.0, we focus our efforts into developing autonomous
control algorithm and system to control a robot’s movement without external reference (e.g. wall, line, etc.) in 2D space. 
Our control parameters include the robot's:
* **DIRECTION**
* **TRAVEL DISTANCE**
* **COORDINATE IN SPACE**

Note that this robot is not capable of sensing and responding to external stimuli. It is able to move in two ways: going straight for a
specified distance and turning on spot for a specified angle. All movements are pre-programmed parameters. 

### Area:
Two-wheeled drive

### Input Sensors:
* MPU 9250
* Rotary Encoder B106

### Research aspects:
1. ***MPU 9250 sensor***
    
    We choose the MPU 9250 because it serves as the robot’s absolute positioning reference. The MPU 9250 is a combination of 3 inertial sensors, the Accelerometer, Gyroscope & Magnetometer. It was able to give us the yaw, pitch and roll angle of the robot with its orientation based of the Earth’s magnetic heading. 
    
    From the MPU 9250 we got the x,y,z orientation of the robot in terms of the 3 inertial sensors as raw data. We fuse and filter the raw data using the Madgwick Filter to get the yaw, pitch and roll angle of the robot. Since our robot only moves in the xy-plane, we only need to concern about the yaw angle here.
    
    We manage to get a pretty accurate yaw angle reading with a fluctuation range of +-1 after some manual calibration of the robot.
    
2. ***Rotary Encoder B106***

    We used rotary encoder as our local reference, where we were able to provide a data feedback for the system so that the robot “knows” its wheel condition, to be more precise by how much the wheel has turned within a time constant. The measurement property is known as “ticks”, and from this raw data we will be able to derive and control various parameters with high precision such as the wheel speed, travel distance, and angle.
    
    We chose the B106 Rotary Encoder as it gives a high resolution of up to 500 ticks per revolution.
    
    To process the encoder ticks count into more useful control parameters, we develop a motion equation to derive the travel distance and angle of robot. It’s a pretty basic set of equation using some trigonometric properties.
    
    `void compute(){
  change_left =input_left -last_input_left ;
  last_input_left =input_left;
  change_right=input_right-last_input_right;
  last_input_right=input_right;
  double s_left =get_curve_length(change_left) ;
  double s_right=get_curve_length(change_right);
  psi+=(s_right-s_left)/(robot_length); 
  double distance=(s_left+s_right)/2;
  double new_psi=psi;
  if(psi<0) new_psi=2*Pi-psi;
    x += distance*cos(new_psi); 
    y += distance*sin(new_psi);
}`
 



* [5 Product Create Form](www.google.com)
