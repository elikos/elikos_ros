#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <string>

const std::string ELIKOS_ARENA_ORIGIN = "elikos_arena_origin";
const std::string ELIKOS_LOCAL_ORIGIN = "elikos_local_origin";
const std::string ELIKOS_FCU = "elikos_fcu";
const std::string ELIKOS_VISION = "elikos_vision";
const std::string ELIKOS_ATTITUDE = "elikos_attitude";

bool isInit_ = false;
bool initialize(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  isInit_ = true;
  return true;
}

tf::Quaternion attitude_;
tf::Quaternion attitude_offset_;

void imu_callback(const sensor_msgs::Imu::ConstPtr& input)
{
	tf::quaternionMsgToTF(input->orientation, attitude_);
}

int main(int argc, char* argv[])
{
  ros::init( argc, argv, "elikos_origin_init" );

  ros::NodeHandle n;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform arenaOriginTransform;
  ros::Subscriber sub_;
  sub_ = n.subscribe("/vn100/imu/imu", 1, imu_callback);


  while (!tf_listener_.canTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0))) 
  {
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.17)), 
          ros::Time::now(), ELIKOS_LOCAL_ORIGIN, ELIKOS_VISION));
  }

  tf::StampedTransform initialFcu;
  try {
    tf_listener_.waitForTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time::now(), ros::Duration(5.0));
    tf_listener_.lookupTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0), initialFcu);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Origin init failed!!!! Exception : %s",ex.what());
  }

  bool lookupDone = false;

  ros::ServiceServer service = n.advertiseService("elikos_origin_init", initialize);
  

  ros::Rate r(10);
  while(ros::ok())
  {
    if(isInit_)
    {
      if(!lookupDone)
      {
        try {
          // Prevent console spam.
          tf_listener_.waitForTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0), ros::Duration(1.0));
    			tf_listener_.lookupTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0), arenaOriginTransform);
          tf::Vector3 origin = initialFcu.getOrigin();
          origin.setZ(0);
          tf::Quaternion rotation = initialFcu.getRotation();
          double yaw = tf::getYaw(rotation);
          rotation.setRPY(0,0,yaw);
          ROS_INFO_STREAM("Initialisation : Yaw diff is : "<<yaw);
          arenaOriginTransform.setOrigin(origin);
          arenaOriginTransform.setRotation(rotation);
          lookupDone = true;
          attitude_offset_ = attitude_;
    		}
    		catch (tf::TransformException &ex) {
    			ROS_ERROR("Origin init failed!!!! Exception : %s",ex.what());
          tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.17)), 
              ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_VISION));
    		}
      }
      tf_broadcaster_.sendTransform(tf::StampedTransform(arenaOriginTransform.inverse(), ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_LOCAL_ORIGIN));
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(attitude_offset_.inverse() * attitude_, tf::Vector3()), ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_ATTITUDE));
    }
    else
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_LOCAL_ORIGIN));
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
