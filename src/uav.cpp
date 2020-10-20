#include "ros/ros.h"
#include "nav_msgs/Odometry.h" 
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" 
#include <iostream>
#include <math.h>
#include <time.h>
#include <string>
#include <std_msgs/Float32.h>	
#include <tf/tf.h>
#include <queue>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "fl/Headers.h"


using namespace fl;

#define NUMBER_OF_POINTS_SPIRAL 720
#define RAIO 5
#define HEIGHT 10


#define WAIT_START 0
#define MOVE 1
#define INSPECT 2
#define LAND 3
#define STOP 4
#define FIM 5

//#define DEBUG_POUSO
//#define DEBUG_DESVIO
//#define DEBUG_DRONE
using namespace std;

tf::Pose pose2;

float yaw_angle;
float position_x=0.1, difpos_x;
float position_y=0.1, difpos_y;
double orientation_w;
double delta =0;
float velocidade_x=0;
int tecla;
geometry_msgs::Point poseDrone;

ros::Publisher pubCmd; 
ros::Publisher pubCmdRobot;
ros::Subscriber sub_odom;
ros::Subscriber sub_marker;
ros::Subscriber sub_lidar_robot;
ros::Subscriber sub_goal;



float dist[16];
sensor_msgs::LaserScan lidar;
sensor_msgs::LaserScan hokuyo;

ar_track_alvar_msgs::AlvarMarkers markers;
geometry_msgs::PoseStamped poseMarker;

geometry_msgs::Pose next_point;


sensor_msgs::Image image;
//ros::Publisher pubImage;


geometry_msgs::PoseStamped posRobo;
geometry_msgs::Point odomDrone;

bool isPouso=false;
bool start=false;

float menor_leitura;

geometry_msgs::Point goal;


struct coordinate{
	float x;
	float y;
	float z;
};

struct hokuyo_engine{
	fl::InputVariable* inputX;
	fl::InputVariable* inputY;
	fl::InputVariable* inputZ;
	fl::OutputVariable* outputX;
	fl::OutputVariable* outputY;
	fl::OutputVariable* outputZ;
	fl::Engine* engine;
};

hokuyo_engine hokEng;


void targetCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& input){
	markers.header=input->header;
	markers.markers=input->markers;
	for(int i=0;i<markers.markers.size();i++){
		if(abs(markers.markers[0].pose.pose.position.z)>0){
			poseMarker=markers.markers[0].pose;
			ROS_INFO("X= %f, Y= %f, Z= %f", poseMarker.pose.position.x,  poseMarker.pose.position.y,  poseMarker.pose.position.y);
		}
	}

//	ROS_INFO("%ld \n", markers.markers.size());
}









void odomDroneCallback(const nav_msgs::Odometry::ConstPtr& msg){
	odomDrone.x=msg->pose.pose.position.x;
	odomDrone.y=msg->pose.pose.position.y;
	odomDrone.z=msg->pose.pose.position.z;
	start=true;
}


void odomRobotCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::Pose poseOdom;
	tf::poseMsgToTF(msg->pose.pose, poseOdom);
	goal.x = poseOdom.getOrigin().getX();
	goal.y = poseOdom.getOrigin().getY();
	goal.z = poseOdom.getOrigin().getZ();

}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg) // quando o subcallback estiver mensagem, ele vai entrar nesta funcao que rece as msg do tipo laser scan
{     
	goal.x=msg->x;
	goal.y=msg->y;
	goal.z=msg->z;
}



void init_fuzzy_hokuyo(){
	fl::Engine* engine = new fl::Engine;
	engine->setName("Controle");

	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setEnabled(true);
	inputVariable1->setName("X");
	inputVariable1->setRange(-1.000, 1.000);
	inputVariable1->addTerm(new fl::ZShape("FarLeft", -1.000, -0.264));
	inputVariable1->addTerm(new fl::Gaussian("MediumLeft", -0.503, 0.179));
	inputVariable1->addTerm(new fl::Gaussian("Close", -0.005, 0.159));
	inputVariable1->addTerm(new fl::Gaussian("MediumRight", 0.512, 0.179));
	inputVariable1->addTerm(new fl::Sigmoid("FarRight", 0.711, 11.005));
	engine->addInputVariable(inputVariable1);

	fl::InputVariable* inputVariable2 = new fl::InputVariable;
	inputVariable2->setEnabled(true);
	inputVariable2->setName("Y");
	inputVariable2->setRange(-1.000, 1.000);
	inputVariable2->addTerm(new fl::ZShape("FarLeft", -1.000, -0.264));
	inputVariable2->addTerm(new fl::Gaussian("MediumLeft", -0.503, 0.179));
	inputVariable2->addTerm(new fl::Gaussian("Close", -0.005, 0.159));
	inputVariable2->addTerm(new fl::Gaussian("MediumRight", 0.512, 0.179));
	inputVariable2->addTerm(new fl::Sigmoid("FarRight", 0.711, 11.005));
	engine->addInputVariable(inputVariable2);

	fl::InputVariable* inputVariable3 = new fl::InputVariable;
	inputVariable3->setEnabled(true);
	inputVariable3->setName("Z");
	inputVariable3->setRange(-1.000, 1.000);
	inputVariable3->addTerm(new fl::ZShape("FarLeft", -1.000, -0.264));
	inputVariable3->addTerm(new fl::Gaussian("MediumLeft", -0.503, 0.179));
	inputVariable3->addTerm(new fl::Gaussian("Close", -0.005, 0.159));
	inputVariable3->addTerm(new fl::Gaussian("MediumRight", 0.512, 0.179));
	inputVariable3->addTerm(new fl::Sigmoid("FarRight", 0.711, 11.005));
	engine->addInputVariable(inputVariable3);

	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setEnabled(true);
	outputVariable1->setName("outX");
	outputVariable1->setRange(-1.000, 1.000);
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
	outputVariable1->setDefuzzifier(new fl::Bisector(200));
	outputVariable1->setDefaultValue(0.000);
	outputVariable1->setLockValidOutput(false);
	outputVariable1->setLockOutputRange(false);
	outputVariable1->addTerm(new fl::ZShape("FarLeft", -1.000, -0.264));
	outputVariable1->addTerm(new fl::Gaussian("MediumLeft", -0.503, 0.179));
	outputVariable1->addTerm(new fl::Gaussian("Close", -0.005, 0.159));
	outputVariable1->addTerm(new fl::Gaussian("MediumRight", 0.512, 0.179));
	outputVariable1->addTerm(new fl::Sigmoid("FarRight", 0.711, 11.005));
	engine->addOutputVariable(outputVariable1);

	fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
	outputVariable2->setEnabled(true);
	outputVariable2->setName("outY");
	outputVariable2->setRange(-1.000, 1.000);
	outputVariable2->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
	outputVariable2->setDefuzzifier(new fl::Bisector(200));
	outputVariable2->setDefaultValue(0.000);
	outputVariable2->setLockValidOutput(false);
	outputVariable2->setLockOutputRange(false);
	outputVariable2->addTerm(new fl::ZShape("FarLeft", -1.000, -0.264));
	outputVariable2->addTerm(new fl::Gaussian("MediumLeft", -0.503, 0.179));
	outputVariable2->addTerm(new fl::Gaussian("Close", -0.005, 0.159));
	outputVariable2->addTerm(new fl::Gaussian("MediumRight", 0.512, 0.179));
	outputVariable2->addTerm(new fl::Sigmoid("FarRight", 0.711, 11.005));
	engine->addOutputVariable(outputVariable2);

	fl::OutputVariable* outputVariable3 = new fl::OutputVariable;
	outputVariable3->setEnabled(true);
	outputVariable3->setName("outZ");
	outputVariable3->setRange(-1.000, 1.000);
	outputVariable3->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
	outputVariable3->setDefuzzifier(new fl::Bisector(200));
	outputVariable3->setDefaultValue(0.000);
	outputVariable3->setLockValidOutput(false);
	outputVariable3->setLockOutputRange(false);
	outputVariable3->addTerm(new fl::ZShape("FarLeft", -1.000, -0.264));
	outputVariable3->addTerm(new fl::Gaussian("MediumLeft", -0.503, 0.179));
	outputVariable3->addTerm(new fl::Gaussian("Close", -0.005, 0.159));
	outputVariable3->addTerm(new fl::Gaussian("MediumRight", 0.512, 0.179));
	outputVariable3->addTerm(new fl::Sigmoid("FarRight", 0.711, 11.005));
	engine->addOutputVariable(outputVariable3);

	fl::RuleBlock* ruleBlock = new fl::RuleBlock;
	ruleBlock->setEnabled(true);
	ruleBlock->setName("Rule");
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setActivation(new fl::AlgebraicProduct);
	ruleBlock->addRule(fl::Rule::parse("if X is FarLeft then outX is FarRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if X is FarRight then outX is FarLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if X is Close then outX is Close ", engine));
	ruleBlock->addRule(fl::Rule::parse("if X is MediumRight then outX is MediumLeft ", engine));
	ruleBlock->addRule(fl::Rule::parse("if X is MediumLeft then outX is MediumRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if Y is FarLeft then outY is FarRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if Y is FarRight then outY is FarLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if Y is Close then outY is Close ", engine));
	ruleBlock->addRule(fl::Rule::parse("if Y is MediumRight then outY is MediumLeft ", engine));
	ruleBlock->addRule(fl::Rule::parse("if Y is MediumLeft then outY is MediumRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if Z is FarLeft then outZ is FarRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if Z is FarRight then outZ is FarLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if Z is Close then outZ is Close ", engine));
	ruleBlock->addRule(fl::Rule::parse("if Z is MediumRight then outZ is MediumLeft ", engine));
	ruleBlock->addRule(fl::Rule::parse("if Z is MediumLeft then outZ is MediumRight", engine));
	engine->addRuleBlock(ruleBlock);


//---------------------------------------------------------------------------------------------------------------------------------
	hokEng.inputX = inputVariable1;
	hokEng.inputY = inputVariable2;
	hokEng.inputZ = inputVariable3;
	hokEng.outputX = outputVariable1;
	hokEng.outputY = outputVariable2;
	hokEng.outputZ = outputVariable3;
	hokEng.engine = engine;

}







geometry_msgs::Twist controle_drone(){
	geometry_msgs::Twist result;
	ros::spinOnce();
	float distX;
	float distY;
	float distZ;
	ROS_WARN("Starting Control");
	distX=(goal.x-odomDrone.x)/5;
	if(distX>1){
		distX=1;
	}
	else if(distX<-1){
		distX=-1;
	}
	distY=(goal.y-odomDrone.y)/5;
	if(distY>1){
		distY=1;
	}
	else if(distY<-1){
		distY=-1;
	}
	distZ=(goal.z-odomDrone.z)/5;
	if(distZ>1){
		distZ=1;
	}
	else if(distZ<-1){
		distZ=-1;
	}
	ROS_WARN("Starting Fuzzy");
	//Set inputs#include "geometry_msgs/Twist.h"
	hokEng.inputX->setInputValue(distX);
	hokEng.inputY->setInputValue(distY);
	hokEng.inputZ->setInputValue(-distZ);
	//Start fuzzy
	ROS_WARN("Processing Fuzzy");
	hokEng.engine->process();
	ROS_WARN("Output Fuzzy");
	//Defuzzification
	fl::scalar outputX = hokEng.outputX->defuzzify();
	fl::scalar outputY = hokEng.outputY->defuzzify();
	fl::scalar outputZ = hokEng.outputZ->defuzzify();


	result.angular.x=0;
	result.angular.y=0;
	result.angular.z=0;
	result.linear.x=outputX;
	result.linear.y=outputY;
	result.linear.z=outputZ;
	return result;
}


// Main function
int main(int argc, char **argv) {

		// ROS publisher and subscriber initialization

	ros::init(argc, argv, "uav_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	geometry_msgs::Twist msg;

	ROS_WARN("Starting node");

//		sub_marker = n.subscribe("/ar_pose_marker", 1000, targetCallback);
	sub_odom = n.subscribe("/bebop/odometry/filtered", 1000, odomDroneCallback);
	sub_goal = n.subscribe("/goal", 1000, goalCallback);
	pubCmd = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1000);

	ros::topic::waitForMessage<nav_msgs::Odometry>("/bebop/odometry/filtered");

//	ros::topic::waitForMessage<geometry_msgs::Point>("/goal");
	ros::Subscriber subOdomRobot = n.subscribe("/groundRobot/odom", 1000, odomRobotCallback);

	init_fuzzy_hokuyo();
	if (ros::ok()) {
		ros::spinOnce();
			//  Control loop

			// Variable init
		int interation=0, round=0;       
		float posdesejada[2], oridesejada, dist=99, erroorie=99, last_erroorie=0, last_dist=0;

		ros::spinOnce();
		loop_rate.sleep();
		ROS_WARN("Starting main loop");
		while(ros::ok()){
			ros::spinOnce();
			msg=controle_drone();
			if(!(isnan(msg.linear.x) || isnan(msg.linear.y) || isnan(msg.linear.z)))
				pubCmd.publish(msg);
			loop_rate.sleep();
		}
	}
	return 0;
}