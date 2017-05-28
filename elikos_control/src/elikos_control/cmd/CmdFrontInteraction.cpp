#include "CmdFrontInteraction.h"

CmdFrontInteraction::CmdFrontInteraction(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    cmdPriority_ = PriorityLevel::INTERACTING;
    cmdCode_ = 2;
    
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 0.0 }));
    targetPosition_.child_frame_id_ = MAV_FRAME;
    targetPosition_.frame_id_ = WORLD_FRAME;
}

CmdFrontInteraction::~CmdFrontInteraction()
{
    int i = 0;
}

void CmdFrontInteraction::execute()
{
    interactionStatus_ = InteractionState::LANDING;
    ros::Rate rate(30.0);

    tf::Vector3 groundPosition = targetPosition_.getOrigin();
    groundPosition.setZ(HIGH_OF_GROUND);
    groundPosition.setY(cmdDestination_.position.y);
    groundPosition.setX(cmdDestination_.position.x);
    targetPosition_.setOrigin(groundPosition);

    
    while (interactionStatus_ != InteractionState::DONE)
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
                // TODO faire une pause au sol
                tf::Vector3 securityPosition = targetPosition_.getOrigin();
                securityPosition.setZ( 1.0 );
                targetPosition_.setOrigin(securityPosition);
            }
        }
    }
}

void CmdFrontInteraction::abort()
{
}

void CmdFrontInteraction::ajustement() //Paramètre à recevoir : XY devant le robot au sol
{
    //TODO : Faire l'update de la position devant le robot
}