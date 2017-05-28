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
    virtual void ajustement();

    void publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint);


private:
    CmdTravel() = delete;

    const double THRESHOLD = 0.05;  //TODO fichier de config

    geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint_;
    tf::StampedTransform lastPosition_;
    
};

#endif /// CMD_TRAVEL_H
