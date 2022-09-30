// This file takes the angular velocities (from /synchronized_motor_speed) of the 4 motors.
// Then it publishes the velocity of the CoG of the robot and its angular velocity on /estimated_v_w.

#include "ros/ros.h"
#include <robotics_hw1/FourSpeeds.h>    // custom message that will contain the synchronized speeds and the header
#include "geometry_msgs/TwistStamped.h"        




class differential_drive_velocities {

public: 

    // mechanical specifications from https://www.agilex.ai/upload/files/44_2104.pdf

    // Declaration of the variables
    float radious = 0.1575;
    float gearbox_ratio;
    
    double time, time_sec, time_nsec;       // auxiliary variables time

    float estimated_baseline = 0;           // The value will be passed using .getParam()

    geometry_msgs::TwistStamped velocities;

    float v_fl = 0; // front left velocity
    float v_rl = 0; // rear left velocity
    float v_l = 0;

    float v_fr = 0; // front right velocity
    float v_rr = 0; // rear right velocity
    float v_r = 0;

    float v = 0;    // linear velocity of the CoG of the robot
    float w = 0;    // angular velocity of the robot

private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub; 
  



public:     

  // Constructor
  differential_drive_velocities(){ 
        n.getParam("/estimated_baseline", estimated_baseline);   // The value of estimated baseline is taken from a ros parameter
        sub = n.subscribe("/synchronized_motor_speed", 1000, &differential_drive_velocities::callback_sub, this);
        pub = n.advertise<geometry_msgs::TwistStamped>("/estimated_v_w", 1000);
        n.getParam("/estimated_gearbox", gearbox_ratio);
  }





  // Callback : when a message arrives, it extracts the four sinchronized speeds from it and computes v and w,
  //            then it generates and publishes the final message
  void callback_sub(const robotics_hw1::FourSpeeds::ConstPtr& msg){
    
    // Extraction of the four speeds
    v_fl = msg->speed_fl/60*2*3.14*radious/gearbox_ratio;
    v_rl = msg->speed_rl/60*2*3.14*radious/gearbox_ratio;
    v_fr = msg->speed_fr/60*2*3.14*radious/gearbox_ratio;
    v_rr = msg->speed_rr/60*2*3.14*radious/gearbox_ratio;


    // Computation of v and w
    v_l = (v_fl + v_rl)/2;  // mean left velocity (positive when going back)
    v_r = (v_fr + v_rr)/2;  // mean right velocity (positive when going forward)

    v = (v_r - v_l)/2;  // remember the sign convention of v_l
    w = (v_r + v_l)/estimated_baseline;  // remember the sign convention of v_l




    // Creation of the final message
    time_sec = msg->header.stamp.sec;
    time_nsec = msg->header.stamp.nsec;

    velocities.twist.linear.x = v;
    velocities.twist.linear.y = 0;      // The robot has speed only on his x and angular speed only around z
    velocities.twist.linear.z = 0;


    velocities.twist.angular.x = 0;
    velocities.twist.angular.y = 0;
    velocities.twist.angular.z = w;

    velocities.header.stamp.sec = time_sec;
    velocities.header.stamp.nsec = time_nsec;
    time = time_sec + time_nsec / 1000000000;           // This variable is used just to print

    velocities.header.seq = msg->header.seq;
    velocities.header.frame_id = msg->header.frame_id;

    pub.publish(velocities);


    ROS_INFO("[DIFFERENTIAL_DRIVE_VELOCITIES] I published the velocities v: %f and w:%f",v,w);
    ROS_INFO(" time = %f \n", time);
  }


};





int main(int argc, char **argv) {
    ros::init(argc, argv, "differential_drive_velocities");

    differential_drive_velocities my_differential_drive_velocities;

    ROS_INFO("differential_drive_velocities CREATED");
    
    ros::spin();
    
    return 0;
}