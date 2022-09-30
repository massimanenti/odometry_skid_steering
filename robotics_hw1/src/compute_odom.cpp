// This file computes the odometry, starting from the messages published on /estimated_v_w.
// Then, it publishes a message of type nav_msgs/Odometry, called "output", on /final_odometry 
// and broadcasts a TF called "transformStamped".

// Moreover:
//  - provides two services, /reset_pose_to_origin which reset the pose to (0,0) without changing the orientation, 
//    and /reset_pose_to which reset pose to a generic pose described by three ros parameters
//  - provides the possibility to select the integration method (Eulero/Runge-Kutta), even while the bag is running. This is possible thanks to dynamic reconfigure, with the parameter E_RK.
//  - publishes a custom message on /final_odometry_and_method, of type OdometryAndMethod.msg

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>             // type of message for the publisher
#include <geometry_msgs/TwistStamped.h>    // type of message for the subscriber
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int64.h>
#include <dynamic_reconfigure/server.h>
#include <robotics_hw1/parametersConfig.h>

#include <robotics_hw1/Pose.h>                                                       
#include <robotics_hw1/Null.h> 

#include <robotics_hw1/OdometryAndMethod.h> // custom message
#include <std_msgs/String.h>

class compute_odom{  
        
    public:
    ros::NodeHandle n;
    ros::Publisher pub_odom;                
    ros::Subscriber sub_odom;
    ros::Publisher pub_odom_and_method;                                   
   
    nav_msgs::Odometry output;                              // name of the final message of type Odometry
    robotics_hw1::OdometryAndMethod odometry_and_method;    // name of the final message of type OdometryAndMethod

    // Setup for the services
    ros::ServiceServer service_generic_pose;                                    
    ros::ServiceServer service_to_origin;    


    // Declaration of the variables
    float v;
    float w;

    double Ts;
    double t_k1;
    double t_k;
    double time, time_sec, time_nsec;

    float x;
    float y;
    float theta;

    int count = 0;  // fictitious variable used to initialize t_k for k = 0, it is impossible to integrate 
                    // the first time the callback runs because only one value for the speeds is available
    
    int i;          // this variable is used to switch the integration method, it's value will be 0 or 1


    // Setup for the TF
    tf2::Quaternion q;      //quaternion which will be useful later
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::TransformBroadcaster broad;

    // Setup for dynamic reconfigure
    dynamic_reconfigure::Server<robotics_hw1::parametersConfig> server;
    dynamic_reconfigure::Server<robotics_hw1::parametersConfig>::CallbackType f;





    public:
    

    // Constructor
    compute_odom(){
        sub_odom = n.subscribe("/estimated_v_w", 1000, &compute_odom::main_callback, this);
        pub_odom = n.advertise<nav_msgs::Odometry>("/final_odometry", 1000);
        pub_odom_and_method = n.advertise<robotics_hw1::OdometryAndMethod>("/final_odometry_and_method", 1000);

        service_generic_pose = n.advertiseService("/reset_pose_to", &compute_odom::reset_pose, this);               
        service_to_origin = n.advertiseService("/reset_pose_to_origin", &compute_odom::reset_pose_to_origin, this); 
        
        f = boost::bind(&compute_odom::dyn_reconfigure_callback, this, _1, _2);
        server.setCallback(f);
        
        n.getParam("/initial_pose_x", x);  
        n.getParam("/initial_pose_y", y);
        n.getParam("/initial_pose_theta", theta);

    }




    // Method of the service /reset_pose_to, 
    //  it sets the values of x,y,th to the desired one
    bool reset_pose(robotics_hw1::Pose::Request  &req, // pointer to input              
         robotics_hw1::Pose::Response &res) // pointer to output, not needed (no client)
    {                                                                                   
        x = req.x;                                                                      
        y = req.y;                                                                      
        theta = req.theta;                                                              
        ROS_INFO("Pose set to: \nX = %f\nY 0 %f\nTHETA = %f", x, y, theta);             
        return true;                                                                    
    }       
    


    // Method of the service /reset_pose_to_origin, 
    //  it sets the values of x,y, but not th to 0
    bool reset_pose_to_origin(robotics_hw1::Null::Request  &req, // pointer to input, not needed    
         robotics_hw1::Null::Response &res) // pointer to output, not needed (no client)    
    {                                                                                   
        x = 0;                                                                      
        y = 0;                                                                      
        ROS_INFO("Pose set to: \nX = %f\nY 0 %f\nTHETA = %f", x, y, theta);             
        return true;                                                                    
    }                                                                             



    // Callback related to dyn_reconfigure,
    //  it waits until a parameter changes, then it puts the new value of the param E_RK inside the variable "i"
    //  this can be either 0 or 1
    void dyn_reconfigure_callback(robotics_hw1::parametersConfig &config, uint32_t level) {
        i = config.E_RK;
        ROS_INFO("Set i to: %d \n", i);
    }



    // Main_Callback
    // when a new message containing v and w arrives on the topic /estimated_v_w it computes the odometry,
    // creates and publishes two messages (one of type Odometry and another of type OdometryAndMethod)
    // and finally broadcasts the TF related to the "estimated_baselink" reference_frame 
    void main_callback(const geometry_msgs::TwistStampedConstPtr& msg) {

    // Print On Screen
    ROS_INFO ("[COMPUTE_ODOM] \nReceived a message from the /estimated_v_w : v = (%f) and w = (%f)", 
        msg->twist.linear.x, 
        msg->twist.angular.z);



    // Computation of x, y, th
    v = msg->twist.linear.x;
    w = msg->twist.angular.z;

    time_sec = msg->header.stamp.sec;
    time_nsec = msg->header.stamp.nsec;
    t_k1 = time_sec + time_nsec / 1000000000;


    if (count == 0) 
        {t_k = t_k1;   // this "if" is used to initialize t_k at the first iteration (because it will have problem computing Ts = t_k1 - t_k at the first cycle)
                                                      
         count = 1;}; 

    
    Ts = t_k1 - t_k;
    x = x + v*Ts*cos(theta + i * w * Ts/2);
    y = y + v*Ts*sin(theta + i * w * Ts/2);
    theta = theta + w*Ts;

    ROS_INFO("Ts: %f", Ts);
    ROS_INFO("t_k: %f\n t_k1: %f", t_k, t_k1);
    ROS_INFO("i: %d \n", i);
    t_k = t_k1;




    // Generation of an Odometry message called "output"
    output.header.frame_id = msg->header.frame_id;
    output.child_frame_id = "estimated_base_link";

    output.pose.pose.position.x = x;            
    output.pose.pose.position.y = y;
    output.pose.pose.position.z = 0;  

                        
    q.setRPY(0, 0, theta);                      
                                               
    output.pose.pose.orientation.x = q.x();
    output.pose.pose.orientation.y = q.y();
    output.pose.pose.orientation.z = q.z();
    output.pose.pose.orientation.w = q.w();


    output.twist.twist.linear.x = v;            
    output.twist.twist.linear.y = 0;
    output.twist.twist.linear.z = 0;


    output.twist.twist.angular.x = 0;
    output.twist.twist.angular.y = 0;
    output.twist.twist.angular.z = w;

    output.header.seq = msg->header.seq;
    output.header.stamp.sec = time_sec;
    output.header.stamp.nsec = time_nsec;

    pub_odom.publish(output);


 
               
    // TF related to "estimated_baselink"
    transformStamped.header.stamp.sec = time_sec;
    transformStamped.header.stamp.nsec = time_nsec;

    transformStamped.header.frame_id = msg->header.frame_id;
    transformStamped.child_frame_id = output.child_frame_id;                  

    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();


    broad.sendTransform(transformStamped);


    
    ROS_INFO("\nX: %f\nY: %f\nTheta: %f\n",
    output.pose.pose.position.x,
    output.pose.pose.position.y,
    theta);





    // Creation of the final message of type OdometryAndMethod called "odometry_and_method"
    odometry_and_method.odom = output;

    if (i == 0) {

        odometry_and_method.method.data = "euler";

    }
    else odometry_and_method.method.data = "rk";

    pub_odom_and_method.publish(odometry_and_method);

    }

};



    


int main(int argc, char** argv) {
    ros::init(argc, argv, "compute_odom");


    compute_odom my_compute_odom; 

    ROS_INFO("Compute_odom CREATED");


    ros::spin();

    return 0;
}
