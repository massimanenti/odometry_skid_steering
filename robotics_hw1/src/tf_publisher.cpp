// This file is used in order to look on Rviz the real trajectory of the robot (read on /scout_odom)
// This will allow a comparison with the estimated trajectory, computed in compute_odom.cpp.

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

class tf_sub_pub {

public: 
  // Constructor
  tf_sub_pub() {
    sub = n.subscribe("/scout_odom", 1000, &tf_sub_pub::callback, this);
  } 

  // Set a transformation framework which relates "odom" (static framework) with "baselink" (moving according the manufactorer odometry)
  void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0) );
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp,  msg->header.frame_id, msg->child_frame_id));
  }

private:
  ros::NodeHandle n; 
  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Subscriber sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_publisher");
  tf_sub_pub my_tf_sub_pub;
  ROS_INFO("TF_publisher CREATED");
  ros::spin();
  return 0;
}


