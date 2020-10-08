#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
using namespace std;


tf::Pose pose2;
string pioneer_frame_id;
string uav_frame_id;
ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped msg; 
nav_msgs::Odometry robotOdom;


float yaw_angle;
float yaw_corr, yaw_grad;
float position_x=0, difpos_x, position_y=0, difpos_y, position_z = 0, difpos_z;
float rotpos_x, rotpos_y;

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



	if(marker->id ==3)	{

		position_x = (marker->pose.position.x); // Change of variables are necessary to correct the quadrant changes cause the camera downpointing.
		position_y = (marker->pose.position.y);// The same to the signal changes.
		position_z= (marker->pose.position.z);
		tf::poseMsgToTF(marker->pose, pose2);//
		yaw_angle = -(tf::getYaw(pose2.getRotation()));
		yaw_corr=3.1415-yaw_angle;

		ROS_INFO("I heard X: [%f]", position_x);
		ROS_INFO("I heard Y: [%f]", position_y);
		ROS_INFO("I heard Z: [%f]", position_z);
		ROS_INFO("I heard W: [%f]", yaw_angle);

		ros::spinOnce();
	}

		difpos_x=position_x;
		difpos_y=position_z*0.3408-position_y;
		difpos_z=position_z*0.9401;

		rotpos_x=difpos_y*sin(yaw_corr)-difpos_x*cos(yaw_corr);
		rotpos_y=difpos_y*cos(yaw_corr)+position_x*sin(yaw_corr);

		yaw_grad=(yaw_corr*360)/(2*3.1415);



		msg.pose.pose.position.x = robotOdom.pose.pose.position.x+rotpos_x;	
		msg.pose.pose.position.y = robotOdom.pose.pose.position.y+rotpos_y;
		msg.pose.pose.position.z = robotOdom.pose.pose.position.z+difpos_z;
//		pose.pose.orientatonx=yaw_grad; 

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