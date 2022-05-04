# Homework 1
Robotics course project 2022

Team members:
Claudio Galimberti -
Alexandra Iuga -
Riccardo Pomarico - 10661306

# Files description:

- robotVelocity.cpp compute velocities with mecanum wheels kinematics
- robotOdometry.cpp gets the twist from geometry_msgs/TwistStamped, computes and publishes the odometry in /odom
- parametersCalibration.cpp is used to calibrate robot parameters (r, l, w, N) to match GT pose

# Parameters description:

- xPrevious, yPrevious, thetaPrevious are params which indicate the previous position of the robot
- vx, vy, wz are params which indicate a rough estimate of v and ω
- gearRatio and encoderResolution are used for rpm
- the array wheelRpmFromTick is the speed of the wheels, which is expressed in radians
- r, l, w, T and N are the wheel radius, wheel position along x, wheel position along y, gear ratio and the encoders resolution

# TF tree structure:

The TF tree’s father represents the reference system of the environment, and the TF tree’s child, base_link, represents the center of mass of the robot.

# Custom message structure:

- wheelSpeed.msg contains rpm and header of all wheels
- ros custom message

# More info:

Service call:
- type rosservice call /givePose x y theta

Dynamic reconfiguration:

- if you want to select Euler or RK through the drop-down menu, type
rosrun rqt_reconfigure rqt_reconfigure

- if you want to select Euler through command line, you can choose one of these alternatives
rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod 0
rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod Euler

- if you want to select Runge-Kutta through command line, you can choose one of these alternatives
rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod 1
rosrun dynamic_reconfigure dynparam set /robotOdometry integrationMethod RK

Parameters calibration:
The bags must be inside the workspace src and the command is rosrun homework1 parametersCalibration.

# Launch files descriptions:
Type from command line roslaunch homework1 launcher.launch
