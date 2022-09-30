// This file will read one message of type robotics_hw1::Speeds on /speeds_for_baseline
// It will use them in order to estimate the gearbox ratio


#include "ros/ros.h"
#include <robotics_hw1/Speeds.h>

class gearbox_estimator {

public:
    // mechanical specifications from https://www.agilex.ai/upload/files/44_2104.pdf
    float radious = 0.1575;

    float temporary_gearbox_ratio;
    float est_gearbox_ratio;

	int count = 0;  // counter used to check the number of activations of the callback
    int k =0;       // counter to compute the gearbox

    float v_fl = 0; // front left velocity
    float v_rl = 0; // rear left velocity
    float v_l = 0;

    float v_fr = 0; // front right velocity
    float v_rr = 0; // rear right velocity
    float v_r = 0;

    float v_xg = 0;  // linear velocity
    float w_z = 0;  // yaw rate


private:
	ros::NodeHandle n;
  	ros::Subscriber sub;
  
public:
    // Constructor:
    gearbox_estimator(){
        sub = n.subscribe("/speeds_for_baseline", 1000, &gearbox_estimator::callback, this); 
  }


// Callback: it extracts the 6 speeds and computes every time the temporary gearbox ratio
//           finally it updates the mean value of all the temporary gearbox ratio computed untill now
void callback(const robotics_hw1::SpeedsConstPtr& msg){
    v_fl = msg->speed_fl/60*2*3.14*radious;
    v_rl = msg->speed_rl/60*2*3.14*radious;
    v_fr = msg->speed_fr/60*2*3.14*radious;
    v_rr = msg->speed_rr/60*2*3.14*radious;
    v_xg = msg->v_x;
    w_z = msg->ome_z;

    count++;
    ROS_INFO("[GEARBOX_ESTIMATOR] After %d cycles", count);


    v_l = (v_fl + v_rl)/2;  // mean left velocity (positive when the robot moves back) without gearbox ratio
    v_r = (v_fr + v_rr)/2;  // mean right velocity (positive when the robot moves forward) without gearbox ratio


    if (v_xg != 0)  // we need to assure that the division for v_x is feasible
    {   
        k++;

        temporary_gearbox_ratio = (v_r - v_l)/2/v_xg;  // formula to estimate gearbox ratio 

        est_gearbox_ratio = (est_gearbox_ratio*(k - 1) + temporary_gearbox_ratio)/k;   
    }


    ROS_INFO("The estimated gearbox ratio is %f", est_gearbox_ratio);
    ROS_INFO("The temporary gearbox ratio is %f", temporary_gearbox_ratio);
     

    ROS_INFO("VL is %f", v_l);
    ROS_INFO("VR is %f", v_r);
    ROS_INFO("Vx is %f", v_xg);
    ROS_INFO("Wz is %f\n", w_z);
  }


};



int main(int argc, char **argv) {

    ros::init(argc, argv, "gearbox_estimator");
  
    gearbox_estimator my_gearbox_estimator;
  
    ROS_INFO("gearbox_estimator CREATED");

    ros::spin();
  
    return 0;
}