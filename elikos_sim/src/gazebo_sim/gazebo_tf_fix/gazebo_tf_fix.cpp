#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>

const std::string ELIKOS_LOCAL_ORIGIN = "elikos_local_origin";
const std::string ELIKOS_FCU = "elikos_fcu";
const std::string ELIKOS_LOCAL_ORIGIN_SIM = "elikos_local_origin_sim";
const std::string ELIKOS_FCU_SIM = "elikos_fcu_sim";

int main(int argc, char* argv[])
{
  ros::init( argc, argv, "gazebo_tf_fix" );

  ros::NodeHandle n;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform arenaOriginTransform;


  tf::StampedTransform fcuTransform;
  bool canTransform = false;
  while(!canTransform)
  {
    try {
      canTransform = tf_listener_.waitForTransform(ELIKOS_LOCAL_ORIGIN_SIM, ELIKOS_FCU_SIM, ros::Time::now(), ros::Duration(1.0));
    }
    catch (tf::TransformException &ex) {
      ROS_INFO("Gazebo tf fix. Wait for transform. Exception : %s",ex.what());
    }
  }

  ros::Rate r(10);
  while(ros::ok())
  {
    try {
      tf_listener_.lookupTransform(ELIKOS_LOCAL_ORIGIN_SIM, ELIKOS_FCU_SIM, ros::Time(0), fcuTransform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("Gazebo tf fix failed!!!! Exception : %s",ex.what());
    }
    tf_broadcaster_.sendTransform(tf::StampedTransform(fcuTransform, ros::Time::now(), ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU));
    
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
