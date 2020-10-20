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
#define DEBUG_ROBOT

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "fl/Headers.h"
using namespace fl;
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

ros::Publisher pub1; 
ros::Publisher pub2; 
ros::Publisher pubCmdRobot;
ros::Subscriber sub_gps;
ros::Subscriber sub_marker;
ros::Subscriber sub_lidar_robot;
ros::Subscriber sub_hokuyo;
ros::Subscriber subOdomRobot;

sensor_msgs::LaserScan lidar;
sensor_msgs::LaserScan hokuyo;

ar_track_alvar_msgs::AlvarMarkers markers;
geometry_msgs::PoseStamped poseMarker;

sensor_msgs::Image image;


geometry_msgs::PoseStamped posRobo;
geometry_msgs::Point odomDrone;

bool isPouso=false;
bool start=false;

float menor_leitura;

ros::Subscriber sub_goal;
geometry_msgs::Point goal;

void goalCallback(const geometry_msgs::Point::ConstPtr& msg) // quando o subcallback estiver mensagem, ele vai entrar nesta funcao que rece as msg do tipo laser scan
{     
	goal.x=msg->x;
	goal.y=msg->y;
	goal.z=msg->z;
}


struct coordinate{
	float x;
	float y;
	float z;
};


struct robo_obj_engine{
	fl::InputVariable* inputVariable1;
	fl::InputVariable* inputVariable2;
	fl::OutputVariable* outputVariable1;
	fl::OutputVariable* outputVariable2;
	fl::Engine* engine;
};


struct robo_obs_engine{
	fl::InputVariable* inputVariable1;
	fl::InputVariable* inputVariable2;
	fl::InputVariable* inputVariable3;
	fl::InputVariable* inputVariable4;
	fl::OutputVariable* outputVariable1;
	fl::OutputVariable* outputVariable2;
	fl::Engine* engine;
};

robo_obs_engine obsEng;
robo_obj_engine objEng;


void odomRobotCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::Pose poseOdom;
	posRobo.header.stamp = ros::Time::now();
	posRobo.header.frame_id =  "/";
	tf::poseMsgToTF(msg->pose.pose, poseOdom);
	yaw_angle = tf::getYaw(poseOdom.getRotation());
	posRobo.pose.position.x = poseOdom.getOrigin().getX();
	posRobo.pose.position.y = poseOdom.getOrigin().getY();
	posRobo.pose.position.z = poseOdom.getOrigin().getZ();

}


void HokuyoCallback(const sensor_msgs::LaserScan::ConstPtr& input) // quando o subcallback estiver mensagem, ele vai entrar nesta funcao que rece as msg do tipo laser scan
{     
  hokuyo.ranges=input->ranges;
}


void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& input) // quando o subcallback estiver mensagem, ele vai entrar nesta funcao que rece as msg do tipo laser scan
{     
  lidar.ranges=input->ranges;
}





void init_fuzzy_robo_obj(){
fl::Engine* engine = new fl::Engine;
engine->setName("Fuzzyobj");

fl::InputVariable* inputVariable1 = new fl::InputVariable;
inputVariable1->setEnabled(true);
inputVariable1->setName("distance");
inputVariable1->setRange(0.000, 50.000);
inputVariable1->addTerm(new fl::Triangle("near", 0.000, 0.500, 1.000));
inputVariable1->addTerm(new fl::Triangle("medium", 0.000, 1.500, 3.000));
inputVariable1->addTerm(new fl::Trapezoid("far", 2.000, 5.000, 50.000, 50.000));
engine->addInputVariable(inputVariable1);

fl::InputVariable* inputVariable2 = new fl::InputVariable;
inputVariable2->setEnabled(true);
inputVariable2->setName("angle");
inputVariable2->setRange(-3.150, 3.150);
inputVariable2->addTerm(new fl::Triangle("BN", -3.150, -3.150, -1.000));
inputVariable2->addTerm(new fl::Triangle("SN", -2.000, -0.700, 0.000));
inputVariable2->addTerm(new fl::Triangle("QZ", -0.500, 0.000, 0.500));
inputVariable2->addTerm(new fl::Triangle("SP", 0.000, 0.700, 2.000));
inputVariable2->addTerm(new fl::Triangle("GP", 1.000, 3.150, 3.150));
engine->addInputVariable(inputVariable2);

fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
outputVariable1->setEnabled(true);
outputVariable1->setName("vel_x");
outputVariable1->setRange(0.000, 5.000);
outputVariable1->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
outputVariable1->setDefuzzifier(new fl::Bisector(200));
outputVariable1->setDefaultValue(fl::nan);
outputVariable1->setLockValidOutput(false);
outputVariable1->setLockOutputRange(false);
outputVariable1->addTerm(new fl::Triangle("slow", 0.000, 0.500, 1.000));
outputVariable1->addTerm(new fl::Triangle("medium", 0.000, 2.500, 5.000));
outputVariable1->addTerm(new fl::Triangle("fast", 3.000, 5.000, 5.000));
engine->addOutputVariable(outputVariable1);

fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
outputVariable2->setEnabled(true);
outputVariable2->setName("rot_z");
outputVariable2->setRange(-1.000, 1.000);
outputVariable2->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
outputVariable2->setDefuzzifier(new fl::Bisector(200));
outputVariable2->setDefaultValue(fl::nan);
outputVariable2->setLockValidOutput(false);
outputVariable2->setLockOutputRange(false);
outputVariable2->addTerm(new fl::Triangle("right", -1.000, -1.000, 0.000));
outputVariable2->addTerm(new fl::Triangle("qz", -0.200, 0.000, 0.200));
outputVariable2->addTerm(new fl::Triangle("left", 0.000, 1.000, 1.000));
engine->addOutputVariable(outputVariable2);

fl::RuleBlock* ruleBlock = new fl::RuleBlock;
ruleBlock->setEnabled(true);
ruleBlock->setName("");
ruleBlock->setConjunction(new fl::AlgebraicProduct);
ruleBlock->setDisjunction(new fl::AlgebraicSum);
ruleBlock->setActivation(new fl::AlgebraicProduct);
ruleBlock->addRule(fl::Rule::parse("if distance is near and angle is BN then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is near and angle is SN then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is near and angle is QZ then vel_x is slow and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is near and angle is SP then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is near and angle is GP then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is medium and angle is BN then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is medium and angle is SN then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is medium and angle is QZ then vel_x is medium and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is medium and angle is SP then vel_x is medium and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is medium and angle is GP then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is far and angle is BN then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is far and angle is SN then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is far and angle is QZ then vel_x is fast and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is far and angle is SP then vel_x is medium and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if distance is far and angle is GP then vel_x is slow and rot_z is left", engine));
engine->addRuleBlock(ruleBlock);




//---------------------------------------------------------------------------------------------------------------------------------
	objEng.inputVariable1 = inputVariable1;	
	objEng.inputVariable2 = inputVariable2;	
	objEng.outputVariable1 = outputVariable1;
	objEng.outputVariable2 = outputVariable2;
	objEng.engine = engine;

}


void init_fuzzy_robo_obs(){
fl::Engine* engine = new fl::Engine;
engine->setName("");

fl::InputVariable* inputVariable1 = new fl::InputVariable;
inputVariable1->setEnabled(true);
inputVariable1->setName("left");
inputVariable1->setRange(0.000, 5.000);
inputVariable1->addTerm(new fl::Trapezoid("near", 0.000, 0.050, 0.500, 1.700));
inputVariable1->addTerm(new fl::Triangle("medium", 1.000, 2.000, 2.500));
inputVariable1->addTerm(new fl::Trapezoid("far", 2.050, 2.850, 5.000, 5.000));
engine->addInputVariable(inputVariable1);

fl::InputVariable* inputVariable2 = new fl::InputVariable;
inputVariable2->setEnabled(true);
inputVariable2->setName("fleft");
inputVariable2->setRange(0.000, 5.000);
inputVariable2->addTerm(new fl::Trapezoid("near", 0.000, 0.050, 0.500, 1.700));
inputVariable2->addTerm(new fl::Triangle("medium", 1.000, 2.000, 2.500));
inputVariable2->addTerm(new fl::Trapezoid("far", 2.050, 2.850, 5.000, 5.000));
engine->addInputVariable(inputVariable2);

fl::InputVariable* inputVariable3 = new fl::InputVariable;
inputVariable3->setEnabled(true);
inputVariable3->setName("fright");
inputVariable3->setRange(0.000, 5.000);
inputVariable3->addTerm(new fl::Trapezoid("near", 0.000, 0.050, 0.500, 1.700));
inputVariable3->addTerm(new fl::Triangle("medium", 1.000, 2.000, 2.500));
inputVariable3->addTerm(new fl::Trapezoid("far", 2.050, 2.850, 5.000, 5.000));
engine->addInputVariable(inputVariable3);

fl::InputVariable* inputVariable4 = new fl::InputVariable;
inputVariable4->setEnabled(true);
inputVariable4->setName("right");
inputVariable4->setRange(0.000, 5.000);
inputVariable4->addTerm(new fl::Trapezoid("near", 0.000, 0.050, 0.500, 1.700));
inputVariable4->addTerm(new fl::Triangle("medium", 1.000, 2.000, 2.500));
inputVariable4->addTerm(new fl::Trapezoid("far", 2.050, 2.850, 5.000, 5.000));
engine->addInputVariable(inputVariable4);

fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
outputVariable1->setEnabled(true);
outputVariable1->setName("vel_x");
outputVariable1->setRange(-1.000, 5.000);
outputVariable1->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
outputVariable1->setDefuzzifier(new fl::Bisector(200));
outputVariable1->setDefaultValue(fl::nan);
outputVariable1->setLockValidOutput(false);
outputVariable1->setLockOutputRange(false);
outputVariable1->addTerm(new fl::Triangle("slow", -0.300, 0.500, 1.000));
outputVariable1->addTerm(new fl::Triangle("medium", 0.000, 2.500, 5.000));
outputVariable1->addTerm(new fl::Triangle("fast", 3.000, 5.000, 5.000));
outputVariable1->addTerm(new fl::Triangle("N", -1.000, -0.500, 0.000));
engine->addOutputVariable(outputVariable1);

fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
outputVariable2->setEnabled(true);
outputVariable2->setName("rot_z");
outputVariable2->setRange(-1.000, 1.000);
outputVariable2->fuzzyOutput()->setAccumulation(new fl::AlgebraicSum);
outputVariable2->setDefuzzifier(new fl::Bisector(200));
outputVariable2->setDefaultValue(fl::nan);
outputVariable2->setLockValidOutput(false);
outputVariable2->setLockOutputRange(false);
outputVariable2->addTerm(new fl::Triangle("right", -1.000, -0.500, 0.000));
outputVariable2->addTerm(new fl::Triangle("qz", -0.100, 0.000, 0.100));
outputVariable2->addTerm(new fl::Triangle("left", 0.000, 0.500, 1.000));
engine->addOutputVariable(outputVariable2);

fl::RuleBlock* ruleBlock = new fl::RuleBlock;
ruleBlock->setEnabled(true);
ruleBlock->setName("");
ruleBlock->setConjunction(new fl::AlgebraicProduct);
ruleBlock->setDisjunction(new fl::AlgebraicSum);
ruleBlock->setActivation(new fl::AlgebraicProduct);
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is near and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is near and right is medium then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is near and right is far then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is medium and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is medium and right is medium then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is medium and right is far then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is far and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is far and right is medium then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is near and fright is far and right is far then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is near and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is near and right is medium then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is near and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is medium and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is medium and right is medium then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is medium and right is far then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is far and right is near then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is far and right is medium then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is medium and fright is far and right is far then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is near and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is near and right is medium then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is near and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is medium and right is near then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is medium and right is medium then vel_x is slow and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is medium and right is far then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is far and right is near then vel_x is medium and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is far and right is medium then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is near and fleft is far and fright is far and right is far then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is near and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is near and right is medium then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is near and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is medium and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is medium and right is medium then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is medium and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is far and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is far and right is medium then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is near and fright is far and right is far then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is near and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is near and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is near and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is medium and right is near then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is medium and right is medium then vel_x is slow and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is medium and right is far then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is far and right is near then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is far and right is medium then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is medium and fright is far and right is far then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is near and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is near and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is near and right is far then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is medium and right is near then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is medium and right is medium then vel_x is medium and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is medium and right is far then vel_x is medium and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is far and right is near then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is far and right is medium then vel_x is medium and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if left is medium and fleft is far and fright is far and right is far then vel_x is fast and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is near and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is near and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is near and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is medium and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is medium and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is medium and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is far and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is far and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is near and fright is far and right is far then vel_x is N and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is near and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is near and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is near and right is far then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is medium and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is medium and right is medium then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is medium and right is far then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is far and right is near then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is far and right is medium then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is medium and fright is far and right is far then vel_x is medium and rot_z is right", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is near and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is near and right is medium then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is near and right is far then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is medium and right is near then vel_x is N and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is medium and right is medium then vel_x is medium and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is medium and right is far then vel_x is medium and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is far and right is near then vel_x is slow and rot_z is left", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is far and right is medium then vel_x is medium and rot_z is qz", engine));
ruleBlock->addRule(fl::Rule::parse("if left is far and fleft is far and fright is far and right is far then vel_x is fast and rot_z is qz", engine));
engine->addRuleBlock(ruleBlock);





//---------------------------------------------------------------------------------------------------------------------------------
	obsEng.inputVariable1 = inputVariable1;
	obsEng.inputVariable2 = inputVariable2;	
	obsEng.inputVariable3 = inputVariable3;
	obsEng.inputVariable4 = inputVariable4;
	
	obsEng.outputVariable1 = outputVariable1;
	obsEng.outputVariable2 = outputVariable2;
	obsEng.engine = engine;
}




geometry_msgs::Twist desvia_obs_robo(float left, float fleft, float fright, float right){
	geometry_msgs::Twist output;
	//Set inputs

	obsEng.inputVariable1->setInputValue(left);
	obsEng.inputVariable2->setInputValue(fleft);
	obsEng.inputVariable3->setInputValue(fright);
	obsEng.inputVariable4->setInputValue(right);


	//Start fuzzy
	obsEng.engine->process();
	
	//Defuzzification
	fl::scalar out1 = obsEng.outputVariable1->defuzzify();
	fl::scalar out2 = obsEng.outputVariable2->defuzzify();
	
	output.linear.x =  out1; 
	output.linear.y =  0;   
	output.linear.z =  0; 
	output.angular.x = 0;
	output.angular.y = 0;
	output.angular.z = -out2;
#ifdef DEBUG_DESVIO
	ROS_WARN("------------------------------------------------------------------------------------------");
	cout << obsEng.inputVariable1->fuzzify(left);
	cout << "\n";
	cout << obsEng.inputVariable2->fuzzify(fleft);
	cout << "\n";
	cout << obsEng.inputVariable3->fuzzify(fright);
	cout << "\n";
	cout << obsEng.inputVariable4->fuzzify(right);
	cout << "\n";
		ROS_WARN("------------------------------------------------------------------------------------------");
	ROS_WARN("Entradas: E= %f,FE=  %f, FD= %f, D= %f",   left, fleft, fright, right);
	ROS_WARN("Saidas: X= %f,Y=  %f",   out1, out2);	
#endif

	return output;
}



geometry_msgs::Twist segue_gps_robo(float posDesx, float posDesy){
	geometry_msgs::Twist output;
	float dist;
	float angle;

	dist = sqrt(pow(posDesx-posRobo.pose.position.x,2)+pow(posDesy-posRobo.pose.position.y,2));;
    angle = atan2(posDesy-posRobo.pose.position.y,posDesx-posRobo.pose.position.x) - yaw_angle;
	//Set inputs#include "geometry_msgs/Twist.h"
    cout<< dist<< "\n";
    if(dist<4){
		output.linear.x =  0;   //out2
		output.linear.y =  0;    //out1
		output.linear.z =  0;   //out4
		output.angular.x = 0;
		output.angular.y = 0;
		output.angular.z = 0;
		return output;    	
    }
	objEng.inputVariable1->setInputValue(dist);
	objEng.inputVariable2->setInputValue(angle);
	//Start fuzzy
	objEng.engine->process();
	
	//Defuzzification
	fl::scalar out1 = objEng.outputVariable1->defuzzify();
	fl::scalar out2 = objEng.outputVariable2->defuzzify();
	
	output.linear.x =  out1;   //out2
	output.linear.y =  0;    //out1
	output.linear.z =  0;   //out4
	output.angular.x = 0;
	output.angular.y = 0;
	output.angular.z = out2;


#ifdef DEBUG_ROBOT
	ROS_WARN("------------------------------------------------------------------------------------------");
	ROS_WARN("Entradas: X= %f,Y=  %f",   dist, angle);
	ROS_WARN("Out: X= %f,Y=  %f",   out1, out2);
	ROS_WARN("------------------------------------------------------------------------------------------");
	cout << objEng.inputVariable1->fuzzify(dist);
	cout << "\n";
	cout << objEng.inputVariable2->fuzzify(angle);
	cout << "\n";
#endif



	return output;
}



geometry_msgs::Twist  controle_robot(){
	geometry_msgs::Twist output;
	coordinate aux;
	float right=4.9;
	float left=4.9;
	float fright=4.9;
	float fleft=4.9;


	if(isPouso){
		output.linear.x=0;
		output.linear.y=0;
		output.linear.z=0;
		output.angular.x=0;
		output.angular.y=0;
		output.angular.z=0;
		return output;
	}
	for(int i=0; i<lidar.ranges.size();i++){
    	if(lidar.ranges[i]<10 && lidar.ranges[i]>0.23){
			if(i<lidar.ranges.size()/4){
				if(lidar.ranges[i]<left){
					left=lidar.ranges[i];
				}
			}
			else if(i<2*lidar.ranges.size()/4){
				if(lidar.ranges[i]<fleft){
					fleft=lidar.ranges[i];
				}	
			}
			else if(i<3*lidar.ranges.size()/4){
				if(lidar.ranges[i]<fright){
					fright=lidar.ranges[i];
				}
			}
			else{
				if(lidar.ranges[i]<right){
					right=lidar.ranges[i];
				}

			}
		}
	}
	aux.x=goal.x;
	aux.y=goal.y;
	aux.z=goal.z;

	if(fleft < 3  ||right < 1.5 || left<1.5 || fright < 3)
		return desvia_obs_robo(left, fleft,  fright, right);
	else{ 
		cout<<"x= "<<aux.x<<"   y= "<<aux.y;
		return segue_gps_robo(aux.y, aux.x);
	}
}



int main(int argc, char **argv) {

		// ROS publisher and subscriber initialization

	ros::init(argc, argv, "robot");
	ros::NodeHandle n;
	ros::Rate loop_rate(3);

	ROS_WARN("Starting node");

	
	sub_lidar_robot = n.subscribe("/scan", 1000, LidarCallback);
	subOdomRobot = n.subscribe("/groundRobot/odom", 1000, odomRobotCallback);
	sub_goal = n.subscribe("/robot_goal", 1000, goalCallback);
	pubCmdRobot = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	ros::topic::waitForMessage<nav_msgs::Odometry>("/groundRobot/odom");
	ros::topic::waitForMessage<geometry_msgs::Point>("/robot_goal");

	ros::spinOnce();
	ros::Rate wait_rate(1);

	ros::Time lastTime = ros::Time::now();

	init_fuzzy_robo_obj();
	init_fuzzy_robo_obs();



	if (ros::ok()) {
		geometry_msgs::Twist msg_robot;
		int interation=0, round=0;       
		float posdesejada[2], oridesejada, dist=99, erroorie=99, last_erroorie=0, last_dist=0;

			ros::spinOnce();
			ROS_WARN("Starting main loop");

			while(1){
				ros::spinOnce();
				msg_robot=controle_robot();
				cout << "a "<< posRobo.pose.position.x <<","<<	posRobo.pose.position.y << " , "<<posRobo.pose.position.z << "\n";
				ROS_WARN("publicando");
				if(!(isnan(msg_robot.linear.x) || isnan(msg_robot.angular.z))){
					ROS_WARN("not nan");
					pubCmdRobot.publish(msg_robot);
				}
				loop_rate.sleep();
			}
		}

		return 0;
}