#include "CmdStandBy.h"

CmdStandBy::CmdStandBy(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)
{
    cmdPriority_ = PriorityLevel::ALWAYS_ABORTABLE;
    cmdCode_ = 5;
    
    currentPosition_.frame_id_ = WORLD_FRAME;
    currentPosition_.child_frame_id_ = MAV_FRAME;
    tf::Transform initialTransform;
    initialTransform.setOrigin({ 0.0, 0.0, 0.0 });
    initialTransform.setRotation({ 0.0, 0.0, 0.0, 1.0 });
    currentPosition_.setData(initialTransform);
}

CmdStandBy::~CmdStandBy()
{
}

void CmdStandBy::execute()
{
    ROS_INFO("Started standby command");
    isAborted_ = false;
    // TODO: Essayer a nouveau si le lookup echoue.
    try {
        tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), currentPosition_);
    } catch(tf::TransformException e) {
        ROS_ERROR("%s",e.what());
    }

    ros::Rate rate(30.0);
    while(ros::ok() && !isAborted_)
    {
        currentPosition_.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(currentPosition_);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Finished standby command");
}

void CmdStandBy::abort()
{
    isAborted_ = true;
}
