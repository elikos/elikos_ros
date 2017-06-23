#ifndef CMD_TRAVEL_H
#define CMD_TRAVEL_H

#ifndef PI
#define PI 3.14159265
#endif

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdTravel : public CmdAbs
{
public:
    CmdTravel(ros::NodeHandle* nh, int id);
    virtual ~CmdTravel();

    virtual void execute();
    virtual void abort();
    void ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory);

    void publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);


private:
    CmdTravel() = delete;
    int stepInTrajectory_;

    double threshold_;
    double max_step_;
    double max_altitude_;
    double dimension_c_;

    geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint_;
    tf::StampedTransform lastPosition_;
    
};

#endif /// CMD_TRAVEL_H
