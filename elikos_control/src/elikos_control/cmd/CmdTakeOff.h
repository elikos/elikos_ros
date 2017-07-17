#ifndef CMD_OFF_BOARD_H
#define CMD_OFF_BOARD_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdTakeOff : public CmdAbs
{
public:
    CmdTakeOff(ros::NodeHandle* nh, int id);
    virtual ~CmdTakeOff();

    virtual void execute();
    virtual void abort();

    void stateCallBack(const mavros_msgs::State::ConstPtr& msg);

private:
    CmdTakeOff() = delete;

    ros::ServiceClient armingClient_;
    ros::ServiceClient setModeClient_;

    ros::Subscriber stateSub_;

    mavros_msgs::State currentState_;
    mavros_msgs::SetMode offbSetMode_;
    mavros_msgs::CommandBool armCmd_;

    ros::Time lastRequest_;

    tf::StampedTransform targetPosition_;
    tf::StampedTransform lastPosition_;

    double threshold_;
    
};

#endif /// CMD_OFF_BOARD_H
