# odometry_skid_steering

The goals of the project are:
I. compute odometry using skid steering (approximate) kinematic
- using Euler and Runge-Kutta integration
- ROS parameter specifies initial pose
II. use dynamic reconfigure to select between integration method
III. write 2 services to reset the odometry to (0,0) or to a pose(x,y,θ)
IV. publish a custom message with odometry value and type of integration

The bag files come from measurements of a real skid-steering robot.

The file "Assignment" is a description of the problem provided by the professor, the file "Useful_information" is useful in order to understand the code present in the folder "robotics_hw1".
