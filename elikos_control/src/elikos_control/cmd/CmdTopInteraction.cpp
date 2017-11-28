#include "CmdTopInteraction.h"

CmdTopInteraction::CmdTopInteraction(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)  
{
    cmdPriority_ = PriorityLevel::INTERACTING;
    cmdCode_ = CmdCode::TOP_INTERACTION;

    takeoff_altitude_ = 1;
    nh_->getParam("/elikos_decisionmaking/takeoff_altitude", takeoff_altitude_);
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, takeoff_altitude_ }));
    targetPosition_.child_frame_id_ = SETPOINT;
    targetPosition_.frame_id_ = WORLD_FRAME;
    
    interaction_altitude_ = 0.1;
    nh_->getParam("/elikos_decisionmaking/interaction_altitude", interaction_altitude_);

	threshold_ = 0.8;
	nh_->getParam("/elikos_decisionmaking/has_reach_destination_threshold", threshold_);
}

CmdTopInteraction::~CmdTopInteraction()
{
    int i = 0;
}

void CmdTopInteraction::execute()
{
    std_msgs::String msg;
    msg.data = "Top Interaction";
    statePubCommand_.publish(msg);

    isAborted_ =false;

    interactionStatus_ = InteractionState::LANDING;
    ros::Rate rate(30.0);

    tf::Vector3 groundPosition = targetPosition_.getOrigin();
    groundPosition.setZ(HIGH_OF_ROBOT);
    groundPosition.setY(cmdDestination_.position.y);
    groundPosition.setX(cmdDestination_.position.x);
    targetPosition_.setOrigin(groundPosition);

    bool has_reached_in_xy = false;
    
    while (interactionStatus_ != InteractionState::DONE  && !isAborted_)
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

        tf::StampedTransform aboveTargetPosition = targetPosition_;
        aboveTargetPosition.getOrigin().setZ(takeoff_altitude_);
        double distance = lastPosition_.getOrigin().distance(aboveTargetPosition.getOrigin());
        if (distance > threshold_ && !has_reached_in_xy)
        {
            aboveTargetPosition.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(aboveTargetPosition);
            rate.sleep();
        }
        else if(lastPosition_.getOrigin().getZ() > interaction_altitude_)
        {
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
            has_reached_in_xy = true;
        }
        else 
        {
            if(interactionStatus_ == InteractionState::HAS_TOUCHED_ROBOT)
            {
                interactionStatus_ = InteractionState::DONE;
            }
            else
            {
                interactionStatus_ = InteractionState::HAS_TOUCHED_ROBOT;
                tf::Vector3 securityPosition = targetPosition_.getOrigin();
                securityPosition.setZ( 1.0 );
                targetPosition_.setOrigin(securityPosition);
                tf_broadcaster_.sendTransform(targetPosition_);
            }
        }
    }
}

void CmdTopInteraction::abort()
{
    isAborted_ = true;
}

void CmdTopInteraction::ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory)
{
    if(lastPosition_.getOrigin().getZ() > threshold_)
    {
        setDestination(destination);
        tf::Vector3 groundPosition = targetPosition_.getOrigin();
        groundPosition.setY(cmdDestination_.position.y);
        groundPosition.setX(cmdDestination_.position.x);
        targetPosition_.setOrigin(groundPosition);
    }
}
