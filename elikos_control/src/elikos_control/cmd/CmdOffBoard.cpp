#include "CmdOffBoard.h"

CmdOffBoard::CmdOffBoard(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    stateSub_ = nh_->subscribe<mavros_msgs::State>("mavros/state", 10, &CmdOffBoard::stateCallBack, this);
    armingClient_ = nh_->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    offbSetMode_.request.custom_mode = "OFFBOARD";
    armCmd_.request.value = true;
    lastRequest_ = ros::Time::now();
}

void CmdOffBoard::stateCallBack(const mavros_msgs::State::ConstPtr& msg)
{
    currentState_ = *msg;
}

void CmdOffBoard::execute()
{

    if (currentState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest_ > ros::Duration(5.0)))
    {
        if (setModeClient_.call(offbSetMode_) && offbSetMode_.response.success)
        {
            ROS_INFO("Offboard enabled");
        }
        lastRequest_ = ros::Time::now();
    } 
    else 
    {
        if (!currentState_.armed && (ros::Time::now() - lastRequest_ > ros::Duration(5.0)))
        {
            if (armingClient_.call(armCmd_) &&
                armCmd_.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            lastRequest_ = ros::Time::now();
        }
    }
}


void CmdOffBoard::abort()
{

}

void CmdOffBoard::ajustement()
{

}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

