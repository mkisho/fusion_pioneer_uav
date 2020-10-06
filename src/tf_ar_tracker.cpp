#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

using namespace std;

string pioneer_frame_id;
string uav_frame_id;


class PoseDrawer {
  public:
    PoseDrawer() : tf_(),  target_frame_("uav") {
      marker_sub_.subscribe(n_, "/poseStamped", 10);
      tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(marker_sub_, tf_, target_frame_, 10);
      tf_filter_->registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
    } ;
  private:
    message_filters::Subscriber<geometry_msgs::PoseStamped> marker_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    //  Callback to register with tf::MessageFilter to be called when transforms are available
    void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& pose_ptr) 
    {
      geometry_msgs::PoseStamped pose_out;
      try 
      {
        tf_.transformPose(target_frame_, *pose_ptr, pose_out);
//        printf("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
 //            pose_out.point.x,
//               pose_out.point.y,
//               pose_out.point.z);
    }
    catch (tf::TransformException &ex){
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };
};


//void poseCallback(const visualization_msgs::Marker::ConstPtr& marker)
//{
//  static tf::TransformBroadcaster br;
//  tf2::Quaternion quat_tf;
//  tf::Transform transform;
//  transform.setOrigin( tf::Vector3(marker->pose.position.x, marker->pose.position.y, marker->pose.position.z));
//  tf2::convert(marker->pose.orientation, quat_tf);
//  transform.setRotation(quat_tf);
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_pioneer", "uav"));
//}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_ar_tracker_node");
  ros::NodeHandle node;
  PoseDrawer pd;
//  ros::Subscriber sub = node.subscribe("/ar_pose_marker", 10, &poseCallback);
  ros::spin();
  return 0;
};