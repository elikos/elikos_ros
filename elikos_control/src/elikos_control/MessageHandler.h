#ifndef MESSAGE_HANDLER
#define MESSAGE_HANDLER

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <elikos_ros/TrajectoryCmd.h>

struct CmdConfig
{
    int id_ = -1;
    uint8_t cmdCode_;
    trajectory_msgs::MultiDOFJointTrajectory cmdTrajectory_;
    geometry_msgs::Pose cmdDestination_;
};

class CmdExecutor;

class MessageHandler
{
public:
    MessageHandler(CmdExecutor& CmdExecutor);
    ~MessageHandler();
    void dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr &input);
    void publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);
    inline CmdConfig getLastCmdConfig() const;

private:
    ros::Subscriber sub_;

    ros::NodeHandle nh_;

    CmdConfig lastReceivedCmd_;
    int lastCmdId_ = 0;

    CmdExecutor& cmdExecutor_;

    MessageHandler() = delete;
};

inline CmdConfig MessageHandler::getLastCmdConfig() const
{
    return lastReceivedCmd_;
}

#endif /// MESSAGE_HANDLER
