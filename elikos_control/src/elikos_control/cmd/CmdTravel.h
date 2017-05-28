#ifndef CMD_TRAVEL_H
#define CMD_TRAVEL_H

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

    void stateCallBack(const mavros_msgs::State::ConstPtr& msg);

private:
    CmdTravel() = delete;

    const double THRESHOLD = 0.05;  //TODO fichier de config

    tf::StampedTransform targetPosition_;
    tf::StampedTransform lastPosition_;
    
};

#endif /// CMD_TRAVEL_H
