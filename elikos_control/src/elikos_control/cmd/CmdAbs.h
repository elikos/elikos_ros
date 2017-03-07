#ifndef CMD_ABS_H
#define CMD_ABS_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <elikos_ros/TrajectoryCmd.h>

class CmdAbs
{
public:


    CmdAbs(ros::NodeHandle* nh, int id);
    virtual ~CmdAbs() = default;

    virtual void execute() = 0;
    virtual void abort() = 0;
    virtual void ajustement() = 0;

    inline void setId(int id);
    inline int getId() const;

    const std::string MAV_FRAME = { "elikos_fcu" };
    const std::string WORLD_FRAME = { "elikos_arena_origin" };

protected:
    int id_ = -1;

    ros::NodeHandle* nh_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

private:
    CmdAbs() = delete;

};

inline int CmdAbs::getId() const
{
    return id_;
}

inline void CmdAbs::setId(int id)
{
    id_ = id;
}

#endif /// CMD_ABS