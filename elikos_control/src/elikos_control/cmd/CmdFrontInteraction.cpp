#include "CmdFrontInteraction.h"

CmdFrontInteraction::CmdFrontInteraction(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id), SLEEP_TIME(4)    
{
    cmdPriority_ = PriorityLevel::INTERACTING;
    cmdCode_ = CmdCode::FRONT_INTERACTION;
    isSleeping_ = false;

    landingClient_ = nh_->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    landingCmd_.request.min_pitch = 0.0;
    landingCmd_.request.yaw = 0.0;
    landingCmd_.request.latitude = 0.0;
    landingCmd_.request.longitude = 0.0;
    landingCmd_.request.altitude = 0.0;
    
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 0.0 }));
    targetPosition_.child_frame_id_ = SETPOINT;
    targetPosition_.frame_id_ = WORLD_FRAME;

    armingClient_ = nh_->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    stateSub_ = nh_->subscribe<mavros_msgs::State>("mavros/state", 10, &CmdFrontInteraction::stateCallBack, this);
    offbSetMode_.request.custom_mode = "OFFBOARD";
    armCmd_.request.value = true;
}

void CmdFrontInteraction::stateCallBack(const mavros_msgs::State::ConstPtr& msg)
{
    currentState_ = *msg;
}

CmdFrontInteraction::~CmdFrontInteraction()
{
    int i = 0;
}

void CmdFrontInteraction::TakeABreak()
{
    std::unique_lock<std::mutex>(sleepLock_);
    isSleeping_ = true;
    sleepCV_.wait_for(sleepLock_, SLEEP_TIME, [this] {return !isSleeping_;});
}


void CmdFrontInteraction::execute()
{
    ROS_ERROR("WATCH OUT!!! FRONT INTERACTION!! :O");
    interactionStatus_ = InteractionState::LANDING;
    ros::Rate rate(30.0);
    isSleeping_ = false;
    isAborted_ = false;

    tf::Vector3 groundPosition = targetPosition_.getOrigin();
    groundPosition.setZ(HIGH_OF_GROUND);
    groundPosition.setY(cmdDestination_.position.y);
    groundPosition.setX(cmdDestination_.position.x);
    targetPosition_.setOrigin(groundPosition);

    
    while (interactionStatus_ != InteractionState::DONE && interactionStatus_ != InteractionState::ASKS_FOR_OFFBOARD && !isAborted_)
    {
        ros::spinOnce();
        try 
        {
            tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
        }

        catch (tf::TransformException e)
        {
            ROS_ERROR("%s",e.what());
        }

        double distance = lastPosition_.getOrigin().distance(targetPosition_.getOrigin());
        if (distance > THRESHOLD)
        {
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
        }
        
        else 
        {
            if(interactionStatus_ == InteractionState::HAS_TOUCHED_GROUND)
            {
                interactionStatus_ = InteractionState::DONE;
            }
            else
            {
                interactionStatus_ = InteractionState::HAS_TOUCHED_GROUND;
                
                landingClient_.call(landingCmd_);
                if(landingCmd_.response.success)
                {
                    //faire une pause au sol. Éventuellement remplacer par signal capteur.
                    TakeABreak();
                    //demander un CmdOffBoard.
                    interactionStatus_ = InteractionState::ASKS_FOR_OFFBOARD;
                    /*while (currentState_.mode == "LAND")
                    {
                        ROS_ERROR("LANDED!");
                    }
                    if (setModeClient_.call(offbSetMode_) && offbSetMode_.response.success)
                    {
                        ROS_ERROR("CmdFrontInteraction : Offboard enabled");
                    } 
                    else 
                    {
                        ROS_ERROR("CmdFrontInteraction : Offboard request failed");
                    }
                    if (armingClient_.call(armCmd_) && armCmd_.response.success)
                    {
                        ROS_ERROR("CmdFrontInteraction : Vehicle armed");
                    }
                    else 
                    {
                        ROS_ERROR("CmdFrontInteraction : Vehicle armed request failed");
                    }*/
                }
                else
                {
                    tf::Vector3 securityPosition = targetPosition_.getOrigin();
                    securityPosition.setZ( 1.0 );
                    targetPosition_.setOrigin(securityPosition);
                }
            }
        }
    }
}

void CmdFrontInteraction::abort()
{
    std::unique_lock<std::mutex>(sleepLock_);
    isSleeping_ = false;
    sleepCV_.notify_one();
    isAborted_ = true;
}

void CmdFrontInteraction::ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory)
{
    std::unique_lock<std::mutex>(sleepLock_);
    if(!isSleeping_ && lastPosition_.getOrigin().getZ() > THRESHOLD)
    {
        setDestination(destination);
        tf::Vector3 groundPosition = targetPosition_.getOrigin();
        groundPosition.setY(cmdDestination_.position.y);
        groundPosition.setX(cmdDestination_.position.x);
        targetPosition_.setOrigin(groundPosition);
    }

}

CmdFrontInteraction::InteractionState CmdFrontInteraction::getStatus()
{
    return interactionStatus_;
}