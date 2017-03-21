#include "CmdOffBoard.h"

CmdOffBoard::CmdOffBoard(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    stateSub_ = nh_->subscribe<mavros_msgs::State>("mavros/state", 10, &CmdOffBoard::stateCallBack, this);
    armingClient_ = nh_->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    offbSetMode_.request.custom_mode = "OFFBOARD";
    armCmd_.request.value = true;
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 2.0 }));
    targetPosition_.child_frame_id_ = "elikos_setpoint";
    targetPosition_.frame_id_ = WORLD_FRAME;
}

CmdOffBoard::~CmdOffBoard()
{
    int i = 0;
}

void CmdOffBoard::stateCallBack(const mavros_msgs::State::ConstPtr& msg)
{
    currentState_ = *msg;
}

void CmdOffBoard::execute()
{
    ros::Rate rate(30.0);

    //send a few setpoints before starting
    for(int i = 0; ros::ok() && i < 100; ++i)
    {
        targetPosition_.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(targetPosition_);
        ros::spinOnce();
        rate.sleep();
    }

    bool isDone = false;
    while (!isDone)
    {
        ros::spinOnce();
        lastRequest_ = ros::Time::now();
        if (currentState_.mode != "OFFBOARD")
        {
            if (setModeClient_.call(offbSetMode_) && offbSetMode_.response.success)
            {
                ROS_INFO("Offboard enabled");
            } 
            else 
            {
                ROS_INFO("Offboard request failed");
            }
            lastRequest_ = ros::Time::now();
        } 
        else if (!currentState_.armed)
        {
            if (armingClient_.call(armCmd_) &&
                armCmd_.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            else 
            {
                ROS_INFO("Vehicle armed request failed");
            }
            lastRequest_ = ros::Time::now();
        }

        try {
            tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
        } catch (tf::TransformException e) {
            ROS_ERROR("%s",e.what());
        }

        double distance = lastPosition_.getOrigin().distance(targetPosition_.getOrigin());
        if (distance > threshold_)
        {
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
        } 
        else 
        {
            isDone = true;
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

