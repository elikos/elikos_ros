#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include "Agent.h"

#include "MessageHandler.h"

namespace ai
{

const std::string MessageHandler::TOPIC = "target_robot_array";
const std::string MessageHandler::MAV_FRAME = "MAV";
const std::string MessageHandler::WORLD_FRAME = "world";
MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler* MessageHandler::getInstance()
{
   if (instance_ == nullptr)
   {
       instance_ = new MessageHandler();
   }
   return  instance_;
}

MessageHandler::MessageHandler()
{
    agent_ = Agent::getInstance();
    sub = nh_.subscribe<elikos_ros::TargetRobotArray>(TOPIC, 1, &MessageHandler::handleMessage, this);
}

void MessageHandler::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MessageHandler::lookForMessages()
{
    ros::spin();
}

void MessageHandler::handleMessage(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    size_t n = input->targets.size();
    const elikos_ros::TargetRobot_<std::allocator<void>>* targets = input->targets.data();
    for (size_t i = 0; i < n; ++i)
    {
        tf::Pose pose;
        tf::poseMsgToTF(targets[i].poseOrigin.pose, pose);
        agent_->updateTarget(targets[i].id, targets[i].color, pose);
    }
    agent_->behave();
}

void MessageHandler::sendDestination(const tf::Vector3& destination)
{
    tf::Transform tf(tf::Quaternion(0.0, 0.0, 0.0, 0.0), destination);
    tf::StampedTransform stf(tf, ros::Time::now(), WORLD_FRAME, MAV_FRAME);
    br_.sendTransform(stf);
}

}
