#include "CmdLanding.h"

CmdLanding::CmdLanding(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    cmdPriority_ = PriorityLevel::LANDING;
    cmdCode_ = 1;
    
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 0.0 }));
    targetPosition_.child_frame_id_ = MAV_FRAME;
    targetPosition_.frame_id_ = WORLD_FRAME;
}

CmdLanding::~CmdLanding()
{
    int i = 0;
}

void CmdLanding::execute()
{
    ros::Rate rate(30.0);
    // TODO: Essayer a nouveau si le lookup echoue.
    try
    {
        tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), targetPosition_);
    }
    catch(tf::TransformException e)
    {
        ROS_ERROR("%s",e.what());
    }

    tf::Vector3 groundPosition = targetPosition_.getOrigin();
    groundPosition.setZ( 0.0 );
    targetPosition_.setOrigin(groundPosition);

    bool isDone = false;
    while (!isDone)
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
        if (distance > threshold_)
        {
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
        }

        else 
        {
            isDone = true;
            // TODO : Il faudrait couper les moteurs...
        }
    }
}

void CmdLanding::abort()
{
}

void CmdLanding::ajustement()
{
}
