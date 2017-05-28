#ifndef CMD_LANDING_H
#define CMD_LANDING_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

#include "CmdAbs.h"

class CmdLanding : public CmdAbs
{
public:
    CmdLanding(ros::NodeHandle* nh, int id);
    virtual ~CmdLanding();

    virtual void execute();
    virtual void abort();
    virtual void ajustement();

private:
    CmdLanding() = delete;

    tf::StampedTransform targetPosition_;
    tf::StampedTransform lastPosition_;

    ros::ServiceClient landingClient_;
    mavros_msgs::CommandTOL landingCmd_;

    double threshold_ = 0.2;
    
};

#endif /// CMD_LANDING_H
