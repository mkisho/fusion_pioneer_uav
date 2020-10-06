#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

using namespace std;

string pioneer_frame_id;
string uav_frame_id;
ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped msg; 
nav_msgs::Odometry robotOdom;

void poseCallback(const visualization_msgs::Marker::ConstPtr& marker){
  //Alterações padrões
  msg.header=marker->header;
  msg.pose.pose=marker->pose;
  msg.header.stamp=ros::Time::now();
  float cov[64]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // translating marker frame to world frame using ground robot odometry
  double distanceMarkerX=msg.pose.pose.position.x;
  double distanceMarkerY=msg.pose.pose.position.y;
  double distanceMarkerZ=msg.pose.pose.position.z;



  msg.pose.pose.position.x=robotOdom.pose.pose.position.x+distanceMarkerX;
  msg.pose.pose.position.y=robotOdom.pose.pose.position.y-distanceMarkerY;
  msg.pose.pose.position.z=robotOdom.pose.pose.position.z+distanceMarkerZ;


  //Publica nova msg
  pose_pub.publish(msg);

}

void groundOdomCallback(const nav_msgs::Odometry::ConstPtr& pointer){
	robotOdom.pose.pose.position= pointer->pose.pose.position;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "poseConverter_node");
  ros::NodeHandle n;
  
  ros::topic::waitForMessage<nav_msgs::Odometry>("/groundRobot/odom");
  
  ros::Subscriber sub = n.subscribe("/visualization_marker", 10, &poseCallback);
  ros::Subscriber sub2 = n.subscribe("/groundRobot/odom", 10, &groundOdomCallback);
  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped >("/bebop/poseStamped", 1);
  ros::spin();
  return 0;
};