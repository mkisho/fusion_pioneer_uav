#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

nav_msgs::Odometry new_odom;
ros::Publisher odom_pub;
void changeFrameId(const nav_msgs::Odometry::ConstPtr& msg)
{
 new_odom.header = msg->header;
 new_odom.header.frame_id = "odom";
 new_odom.child_frame_id =  "base_link";
 new_odom.pose = msg->pose;
 new_odom.twist = msg->twist;
 odom_pub.publish(new_odom);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_tf_name_node");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("/odom_pioneer", 1000);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, changeFrameId);
  int count = 0;
  if (ros::ok())
  {   
    ros::spin();
  }


  return 0;
}

