#include "TargetRobot.h"

namespace ai {

TargetRobot::TargetRobot(uint8_t id, uint8_t color, tf::Vector3 position)
    : id_(id), color_(color)
{
    pose_.setOrigin(position);
}

TargetRobot::~TargetRobot()
{
}

void TargetRobot::prepareForUpdate()
{
    priority_ = 0.0;
    nMissedUpdates_++;
    ROS_ERROR_STREAM("nMissedUpdates_="<<nMissedUpdates_<<" id="<<(int)id_);
}

};
