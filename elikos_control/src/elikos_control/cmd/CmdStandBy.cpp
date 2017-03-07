#include "CmdStandBy.h"

CmdStandBy::CmdStandBy(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)
{
}

CmdStandBy::~CmdStandBy()
{
    int i = 0;
}

void CmdStandBy::execute()
{
    continue_ = true;
    // TODO: Essayer a nouveau si le lookup echoue.
    try {
        tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), currentPosition_);
    } catch(tf::TransformException e) {
    }

    ros::Rate rate(30.0);
    while(ros::ok() && continue_)
    {
        tf_broadcaster_.sendTransform(currentPosition_);
        ros::spinOnce();
        rate.sleep();
    }
}

void CmdStandBy::abort()
{
    continue_ = false;
}

void CmdStandBy::ajustement()
{

}