#include "CmdTopInteraction.h"

CmdTopInteraction::CmdTopInteraction(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)  
{
    cmdPriority_ = PriorityLevel::INTERACTING;
    cmdCode_ = 3;

    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 2.0 }));
    targetPosition_.child_frame_id_ = MAV_FRAME;
    targetPosition_.frame_id_ = WORLD_FRAME;
}

CmdTopInteraction::~CmdTopInteraction()
{
    int i = 0;
}

void CmdTopInteraction::execute()
{
    isAborted_ =false;

    interactionStatus_ = InteractionState::LANDING;
    ros::Rate rate(30.0);

    tf::Vector3 groundPosition = targetPosition_.getOrigin();
    groundPosition.setZ(HIGH_OF_ROBOT);
    groundPosition.setY(cmdDestination_.position.y);
    groundPosition.setX(cmdDestination_.position.x);
    targetPosition_.setOrigin(groundPosition);

    
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

        double distance = lastPosition_.getOrigin().distance(targetPosition_.getOrigin());
        if (distance > THRESHOLD)
        {
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
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
    if(lastPosition_.getOrigin().getZ() > THRESHOLD)
    {
        setDestination(destination);
        tf::Vector3 groundPosition = targetPosition_.getOrigin();
        groundPosition.setY(cmdDestination_.position.y);
        groundPosition.setX(cmdDestination_.position.x);
        targetPosition_.setOrigin(groundPosition);
    }
}
