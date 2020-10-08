#include "ros/ros.h"
#include "nav_msgs/Odometry.h" 
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" 
#include <iostream>
#include <math.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Empty.h>
#include <stdlib.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

using namespace std;

tf::Pose pose2;

geometry_msgs::Pose pose;     

float yaw_angle, pich_angle, row_angle;
float yaw_corr, yaw_grad;

double roll, pitch, yaw;   

float position_x=0, difpos_x, position_y=0, difpos_y, position_z = 0, difpos_z;
float rotpos_x, rotpos_y;

double orientation_w;
double delta =0;
float velocidade_x=0;
float angulo_graus;
int c;
geometry_msgs::Point msg;
std_msgs::Float32 orientation;

ros::Publisher pub1; 
ros::Publisher pub2; 
ros::Publisher pub3; 



void subCallback(const visualization_msgs::Marker::ConstPtr& marker)
{

	pose.orientation.x = (float)marker->id;	
	pose.position.x = marker->pose.position.x;	
	pose.position.y = marker->pose.position.y;
	pose.position.z = marker->pose.position.z;

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

}

int main(int argc, char **argv)
{
// ROS publisher and subscriber initialization
	ros::init(argc, argv, "node1");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<geometry_msgs::Point>("/position", 100);
	ros::Publisher pub2 = n.advertise<std_msgs::Float32>("/orientation", 100);
	ros::Subscriber quaternio = n.subscribe("/visualization_marker", 100, subCallback); 
	ros::Rate loop_rate(20);

	while(ros::ok())
	{    

		difpos_x=position_x;
		difpos_y=position_z*0.3408-position_y;
		difpos_z=position_z*0.9401;

		rotpos_x=difpos_y*sin(yaw_corr)-difpos_x*cos(yaw_corr);
		rotpos_y=difpos_y*cos(yaw_corr)+position_x*sin(yaw_corr);

		yaw_grad=(yaw_corr*360)/(2*3.1415);



		msg.x = rotpos_x;	
		msg.y = rotpos_y;
		msg.z = difpos_z;
		orientation.data=yaw_grad; 

		pub1.publish(msg);
		pub2.publish(orientation);
		loop_rate.sleep(); 

		ros::spinOnce();



	}


	return 0;
}
