#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

std::string tag_name;



void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "uav", tag_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tag_tf_broadcaster");
  tag_name = "ar_marker_3";

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/poseStamped", 10, &poseCallback);

  ros::spin();
  return 0;
};
