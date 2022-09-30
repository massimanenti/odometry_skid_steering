// This file will read one message of type robotics_hw1::Speeds on /speeds_for_baseline
// It will use them in order to estimate the apparent baseline


#include "ros/ros.h"
#include <robotics_hw1/Speeds.h>

class baseline_estimator {

public:
    // mechanical specifications from https://www.agilex.ai/upload/files/44_2104.pdf
    float radious = 0.1575;
    float gearbox_ratio;

	int count = 0;  // counter used to check the number of activations of the callback
    int i = 0;      // counter used to compute the baseline

    float est_base = 0; // this float will be the estimated baseline
    float temporary_base = 0; // this will be the estimated the baseline in one single iteration of the while loop

    float v_fl = 0; // front left velocity
    float v_rl = 0; // rear left velocity
    float v_l = 0;

    float v_fr = 0; // front right velocity
    float v_rr = 0; // rear right velocity
    float v_r = 0;

    float w_z = 0;  // yaw rate


private:
	ros::NodeHandle n;
  	ros::Subscriber sub;
  
public:
    // Constructor:
    baseline_estimator(){
        sub = n.subscribe("/speeds_for_baseline", 1000, &baseline_estimator::callback, this); 
        n.getParam("/estimated_gearbox", gearbox_ratio);
  }

// Callback: it takes the velocities and computed first the temporary baseline and finally updates the average baseline
//           based on the previous values of temporary baseline
void callback(const robotics_hw1::SpeedsConstPtr& msg){

    v_fl = msg->speed_fl/60*2*3.14*radious/gearbox_ratio;
    v_rl = msg->speed_rl/60*2*3.14*radious/gearbox_ratio;
    v_fr = msg->speed_fr/60*2*3.14*radious/gearbox_ratio;
    v_rr = msg->speed_rr/60*2*3.14*radious/gearbox_ratio;
    w_z = msg->ome_z;


    count++;
    ROS_INFO("[BASELINE_ESTIMATOR] After %d cycles", count);


    if (w_z != 0)  // we need to assure that the division for w_z is feasible
    {   
        i++;
        v_l = (v_fl + v_rl)/2;  // mean left velocity (positive when the robot moves back)
        v_r = (v_fr + v_rr)/2;  // mean right velocity (positive when the robot moves forward)

        temporary_base = (v_l + v_r)/w_z;  // formula to estimate apparent baseline in skid steering
        est_base = (est_base*(i - 1) + temporary_base)/i;   
    }


    ROS_INFO("The estimated baseline is %f", est_base);
    ROS_INFO("The temporary baseline is %f", temporary_base);

    ROS_INFO("VL is %f", v_l);
    ROS_INFO("VR is %f", v_r);
    ROS_INFO("Wz is %f\n", w_z);
  }


};




int main(int argc, char **argv) {

    ros::init(argc, argv, "baseline_estimator");
  
    baseline_estimator my_baseline_estimator;
  
    ROS_INFO("baseline_estimator CREATED");

    ros::spin();
  
    return 0;
}