#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <elikos_ros/TrajectoryCmd.h>

#ifndef PI
#define PI 3.14159265
#endif

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr &input);
    void publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
  	double toleranceNextGoal_;
};

#endif /// MESSAGE_HANDLER
