#ifndef CMD_STAND_BY_H
#define CMD_STAND_BY_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdStandBy : public CmdAbs
{
public:
    CmdStandBy(ros::NodeHandle* nh, int id);
    virtual ~CmdStandBy() = default;

    virtual void execute();
    virtual void abort();
    virtual void ajustement();

private:
    CmdStandBy() = delete;
};

#endif /// CMD_OFF_BOARD_H