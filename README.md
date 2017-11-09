# Accurate Robot Motion Control without External Reference System 

This is UTP Petrobot's first major step into autonomous mobile robotics. For V1.0, we focus our efforts into developing autonomous
control algorithm and system to control a robot’s movement without external reference (e.g. wall, line, etc.) in 2D space. 
Our control parameters include the robot's:
* **DIRECTION**
* **TRAVEL DISTANCE**
* **COORDINATE IN SPACE**

Note that this robot is not capable of sensing and responding to external stimuli. It is able to move in only **2 general motions**. All movements are pre-programmed parameters. 

### Area:
Two-wheeled drive

### Input Sensors:
* MPU 9250
* Rotary Encoder B106

### 2 General Motions of robot:
* going straight for a specified distance
* turning on spot for a specified angle and turn direction

### Research aspects:
1. ***MPU 9250 sensor***
    
    We choose the MPU 9250 because it serves as the robot’s absolute positioning reference. The MPU 9250 is a combination of 3 inertial sensors, the Accelerometer, Gyroscope & Magnetometer. It was able to give us the yaw, pitch and roll angle of the robot with its orientation based of the Earth’s magnetic heading. 
    
    From the MPU 9250 we got the x,y,z orientation of the robot in terms of the 3 inertial sensors as raw data. We fuse and filter the raw data using the Madgwick Filter to get the yaw, pitch and roll angle of the robot. Since our robot only moves in the xy-plane, we only need to concern about the yaw angle here.
    
    We manage to get a pretty accurate yaw angle reading with a fluctuation range of +-1 after some manual calibration of the robot.
    
2. ***Rotary Encoder B106***

    We used rotary encoder as our local reference, where we were able to provide a data feedback for the system so that the robot “knows” its wheel condition, to be more precise by how much the wheel has turned within a time constant. The measurement property is known as “ticks”, and from this raw data we will be able to derive and control various parameters with high precision such as the wheel speed, travel distance, and angle.
    
    We chose the B106 Rotary Encoder as it gives a high resolution of up to 500 ticks per revolution.
    
    To process the encoder ticks count into more useful control parameters, we develop a motion equation to derive the travel distance and angle of robot. It’s a pretty basic set of equation using some trigonometric properties. From there, we get the x, y coordinate and the angle of the robot from its initial position.
    
    ```
    void compute(){
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
    }
    ```

3. ***Moving Straight***
    
    The movement of our robot is based on the differential drive theory, where you vary the speed of both wheels to control the robot's heading or turn direction. To reduce swaying in the robot due to physical factors, we've designed a P-regulator according to the PID (Proportional, Integral Derivative) control theory to regulate the motor speed based on the error margin of our input data (yaw angle from MPU) from our target point (target yaw angle value we preset). This enables us to create a more dynamic self-correcting mechanism to minimize large fluctuations in the robot movement.
    
4. ***Turning for an angle***

    We did the robot turning based on the yaw angle data from MPU 9250. We designed a simplified version of PI regulator for this. An aggressive P-regulator is used for big error ranges while a conservative I-regulator is used for small error ranges. This effectively produces a neat correction effect in small error ranges where the accumulation of tiny errors in the I-regulator results in a nice little push of the robot towards its desired target point.
    
5. ***Distance calculation and control***

    From the processed encoder data, we use the x-coordinate as our distance measurement. We ensure that there is a "stopping time" of 500-1000ms after each ***general motion***. We only record the distance at the end of the stopping time to ensure that it is a valid distance, which is recorded after the robot has fully stopped. Our distance calculated from encoder has a deviation error of up to 5% from the actual distance. To make the robot move a specified distance accurately, PID control is again needed to minimize the effect of overshooting caused by inertia and other physics factor.
    
6. ***End Product***

    Now that we have gotten the yaw angle data from MPU, and the travel distance from encoder, we can combine them to make the robot move based on predefined travel distance and turn angle. We can pre-programmed the values of our two parameters into two sets of arrays, and include our array variable in the main loop which goes backs and forth our robot's main function -- go straight for a specified distance, turn for a specified angle, and then go straight again.
    
    Now we can preset our robot to move to any coordinate in space we want through the 2 general motions!
    
### Demo Video:
* [Robot Motion in 2D space](https://www.facebook.com/utppetrobots/videos/1930937540264607/)
