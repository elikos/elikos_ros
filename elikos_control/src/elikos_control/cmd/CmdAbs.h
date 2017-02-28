#ifndef CMD_ABS_H
#define CMD_ABS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <elikos_ros/TrajectoryCmd.h>

class CmdAbs
{
public:
    CmdAbs(ros::NodeHandle* nh);
    virtual ~CmdAbs() = default;

    virtual void execute() = 0;
    virtual void abort() = 0;
    virtual void ajustement() = 0;
    

protected:
    ros::NodeHandle* nh_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;



private:
    CmdAbs() = delete;
};

#endif /// CMD_ABS
