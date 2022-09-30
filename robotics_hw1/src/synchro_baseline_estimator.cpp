// This file has the only purpose of synchronizing the velocities useful to estimate the baseline and the gearbox

// It synchronizes the four speeds (read on the topics /motor_speed_..) and the manufactorer odometry (read on /scout_odom)
// Then, it sends a message of type robotics_hw1::Speeds on /speeds_for_baseline

// _b_e stands for baseline estimator



#include "ros/ros.h"
#include <robotics_hw1/MotorSpeed.h>
#include <robotics_hw1/Speeds.h>            // Custom message, used just for the estimation of the baseline
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>



class synchro_baseline_estimator {  
        
    public:
    ros::NodeHandle n;
    ros::Publisher pub_sync_baseline_estimator;    // publisher of the message filter, it publishes the speed of the wheels syncronized
    robotics_hw1::Speeds msg_sync_b_e;             // msg of the synchronized speeds
    
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_rr;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odometry;  // subscribed to the topic in which manufacturer odometry is published


    typedef message_filters::sync_policies
        ::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry> MySyncPolicy_b_e;

    typedef message_filters::Synchronizer<MySyncPolicy_b_e> Sync_b_e;
    boost::shared_ptr<Sync_b_e> sync_b_e;
    


    public:

    // Constructor:
    synchro_baseline_estimator(){
        sub_motor_fl.subscribe(n, "motor_speed_fl", 1000);
        sub_motor_rl.subscribe(n, "motor_speed_rl", 1000);
        sub_motor_fr.subscribe(n, "motor_speed_fr", 1000);
        sub_motor_rr.subscribe(n, "motor_speed_rr", 1000); 
        sub_odometry.subscribe(n, "scout_odom", 1000);

        pub_sync_baseline_estimator = n.advertise<robotics_hw1::Speeds>("/speeds_for_baseline", 1000);

        sync_b_e.reset(new Sync_b_e(MySyncPolicy_b_e(5), sub_motor_fl, sub_motor_rl, sub_motor_fr, sub_motor_rr, sub_odometry));
        sync_b_e -> registerCallback(boost::bind(&synchro_baseline_estimator::callback, this, _1, _2, _3, _4, _5));
    }


    // Callback: creates a message of type Speeds (custom message) and publishes it on /speeds_for_baseline
    void callback(const robotics_hw1::MotorSpeedConstPtr& msg1, 
                const robotics_hw1::MotorSpeedConstPtr& msg2,
                const robotics_hw1::MotorSpeedConstPtr& msg3,
                const robotics_hw1::MotorSpeedConstPtr& msg4,
                const nav_msgs::OdometryConstPtr& msg5) {


    // Print for debug    
    ROS_INFO ("[SYNCHRO_BASELINE_ESTIMATOR] Published:\n vfl %f\n vrl %f\n vfr %f\n vrr %f\n v_x %f\n wz %f\n", 
       msg1->rpm/60*2*3.14*0.1575, 
       msg2->rpm/60*2*3.14*0.1575,
       msg3->rpm/60*2*3.14*0.1575,
       msg4->rpm/60*2*3.14*0.1575,
       msg5->twist.twist.linear.x,
       msg5->twist.twist.angular.z);

    // Fills every field of msg_sync with the proper velocity, then publishes on /synchronized_motor_speed a single message with the 6 speeds
    msg_sync_b_e.speed_fl = msg1->rpm;
    msg_sync_b_e.speed_rl = msg2->rpm;
    msg_sync_b_e.speed_fr = msg3->rpm;
    msg_sync_b_e.speed_rr = msg4->rpm;
    msg_sync_b_e.v_x = msg5->twist.twist.linear.x;
    msg_sync_b_e.ome_z = msg5->twist.twist.angular.z;

    pub_sync_baseline_estimator.publish(msg_sync_b_e);
    }

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "synchro_baseline_estimator");

    synchro_baseline_estimator my_synchro_baseline_estimator; 

    ROS_INFO("synchro_baseline_estimator CREATED");

    ros::spin();

    return 0;
}