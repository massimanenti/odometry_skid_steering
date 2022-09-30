// This file is used to synchronize the four speeds coming from the topics of the bag (/motor_speed_  )
// Then, it publishes a custom message which contains the four speeds at a given time instant, and the header containing the time instant

#include "ros/ros.h"
#include <robotics_hw1/MotorSpeed.h>
#include <robotics_hw1/FourSpeeds.h>    // custom message that will contain the synchronized speeds
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>

class synchro_pub_sub {  
        
    public:
    ros::NodeHandle n;
    ros::Publisher pub_sync;                // publisher of the message filter, it publishes the speed of the wheels syncronized
    ros::Subscriber sub_manufacturer_odom;  // used in order to take the frame_id from /scout_odom topic of the bag
    robotics_hw1::FourSpeeds msg_sync;      // msg of the synchronized speeds, contains also the frame_id of the manufacturer's odom


    // We define below the 4 subs for the 4 motors
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_motor_rr;



    double time, time_sec, time_nsec;

    typedef message_filters::sync_policies
        ::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;

    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    

    public:    
    // Constructor
    synchro_pub_sub(){
        sub_motor_fl.subscribe(n, "motor_speed_fl", 1000);
        sub_motor_rl.subscribe(n, "motor_speed_rl", 1000);
        sub_motor_fr.subscribe(n, "motor_speed_fr", 1000);
        sub_motor_rr.subscribe(n, "motor_speed_rr", 1000); 

        sub_manufacturer_odom = n.subscribe("/scout_odom", 1000, &synchro_pub_sub::callback2, this);

        // The publisher will publish on a topic called /synchronized_motor_speed

        pub_sync = n.advertise<robotics_hw1::FourSpeeds>("/synchronized_motor_speed", 1000);

        sync.reset(new Sync(MySyncPolicy(5), sub_motor_fl, sub_motor_rl, sub_motor_fr, sub_motor_rr));
        sync -> registerCallback(boost::bind(&synchro_pub_sub::callback, this, _1, _2, _3, _4));
    }
            


    // SECOND CALLBACK:
        // It activates when a msg arrives in the topic /scout_odom and puts its header in the one of the final message

    void callback2(const nav_msgs::Odometry::ConstPtr& msg_odom){
        msg_sync.header.frame_id = msg_odom->header.frame_id;
    }


    // MAIN CALLBACK:
        // Every time that the synchronization happens, the callback will 
        //   -save the stamp.sec and stamp.nsec in time_sec and in time_nsec
        //   -print the time instant and the four speeds on screen
        //   -assemble every field of the custom message msg_sync
        //   -publish the message
    void callback(const robotics_hw1::MotorSpeedConstPtr& msg1, 
                const robotics_hw1::MotorSpeedConstPtr& msg2,
                const robotics_hw1::MotorSpeedConstPtr& msg3,
                const robotics_hw1::MotorSpeedConstPtr& msg4) {


        // Time variables assignment  
        time_sec = msg1->header.stamp.sec;         
        time_nsec = msg1->header.stamp.nsec;      
       
        time = time_sec + time_nsec/1000000000;     // the variable time is used just to print


        // Print for debug
    ROS_INFO ("[SYNCHRO_MOTOR] Received and published 5 messages: time: %f\n, and four speeds: %f and %f and %f and %f", 
        time,
        msg1->rpm, 
        msg2->rpm,
        msg3->rpm,
        msg4->rpm);

    
    // Creation of the final message to be published
    msg_sync.header.stamp.sec = time_sec;
    msg_sync.header.stamp.nsec = time_nsec;
    msg_sync.speed_fl = msg1->rpm;
    msg_sync.speed_rl = msg2->rpm;
    msg_sync.speed_fr = msg3->rpm;
    msg_sync.speed_rr = msg4->rpm;

    msg_sync.header.seq = msg1->header.seq;

    pub_sync.publish(msg_sync);
    }

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "synchro_motor");

    synchro_pub_sub my_synchro_pub_sub; 

    ROS_INFO("Synchro_motor CREATED");

    ros::spin();

    return 0;
}
