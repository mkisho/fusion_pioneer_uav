#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

using namespace std;

string pioneer_frame_id;
string uav_frame_id;
ros::Publisher pose_pub;
nav_msgs::Odometry msg; 
void poseCallback(const nav_msgs::Odometry::ConstPtr& marker){
  msg.header=marker->header;
  msg.pose=marker->pose;
  msg.twist=marker->twist;

  msg.header.stamp=ros::Time::now();
  msg.child_frame_id="uav";
  msg.header.frame_id="odom";
  pose_pub.publish(msg);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "poseConverter_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/bebop/odom/errada", 1, &poseCallback);
  pose_pub = n.advertise<nav_msgs::Odometry >("/bebop/odom", 1000);
  ros::spin();
  return 0;
};