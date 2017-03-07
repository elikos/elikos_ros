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
    int id_ = -1;
    uint8_t cmdCode_;
    trajectory_msgs::MultiDOFJointTrajectory cmdTrajectory_;
};

class CmdExecutor;

class MessageHandler
{
public:
    MessageHandler(ros::NodeHandle* nh);
    ~MessageHandler();
    void dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr &input);
    void publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);
    inline CmdConfig getLastCmdConfig() const;

private:
    ros::Subscriber sub_;

    CmdConfig lastReceivedCmd_;
    int lastCmdId_ = 0;

    MessageHandler() = delete;
};

inline CmdConfig MessageHandler::getLastCmdConfig() const
{
    return lastReceivedCmd_;
}

#endif /// MESSAGE_HANDLER
