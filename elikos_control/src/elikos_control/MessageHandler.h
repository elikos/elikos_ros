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

struct CmdConfig
{
    uint8_t cmdCode_;
    trajectory_msgs::MultiDOFJointTrajectory cmdTrajectory_;
};

class CmdExecutor;

class MessageHandler
{
public:
    MessageHandler();
    ~MessageHandler();
    void dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr &input);
    void publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);
    inline CmdConfig getLastCmdConfig() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    CmdConfig lastReceivedCmd_;
};

inline CmdConfig MessageHandler::getLastCmdConfig() const
{
    return lastReceivedCmd_;
}

#endif /// MESSAGE_HANDLER
