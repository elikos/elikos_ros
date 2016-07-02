#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include "Agent.h"

#include "MessageHandler.h"

namespace ai
{

const std::string MessageHandler::TRGT_TOPIC = "target_robot_array";
const std::string MessageHandler::SETPOINT_TOPIC = "mavros/setpoint/local_position";
const std::string MessageHandler::MAV_TOPIC = "mavros/local_position";
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
    trgtSub_ = nh_.subscribe<elikos_ros::TargetRobotArray>(TRGT_TOPIC, 1, &MessageHandler::handleTrgtMsg, this);
    mavSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(MAV_TOPIC, 1, &MessageHandler::handleMavMsg, this);
    mavPub_ = nh_.advertise<geometry_msgs::PoseStamped>(SETPOINT_TOPIC, 1);
}

void MessageHandler::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MessageHandler::lookForMessages()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        agent_->behave();
        rate.sleep();
    }
}

void MessageHandler::handleTrgtMsg(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    PriorityEvaluationPipeline* pipeline = agent_->getConsiderationPipeline();
    const elikos_ros::TargetRobot_<std::allocator<void>>* targets = input->targets.data();

    size_t n = input->targets.size();
    for (size_t i = 0; i < n; ++i)
    {
        tf::Pose pose;
        tf::poseMsgToTF(targets[i].poseOrigin.pose, pose);
        pipeline->updateTarget(targets[i].id, targets[i].color, pose);
    }
}

void MessageHandler::handleMavMsg(const geometry_msgs::PoseStamped::ConstPtr &input)
{
    tf::Pose pose;
    tf::poseMsgToTF(input->pose, pose);
    agent_->updateQuadRobot(pose);
}

void MessageHandler::sendDestination(const tf::Vector3& destination)
{
    tf::Pose pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), destination);
    tf::Stamped<tf::Pose> stPose(pose, ros::Time::now(), WORLD_FRAME);
    geometry_msgs::PoseStamped msg;
    tf::poseStampedTFToMsg(stPose, msg);
    mavPub_.publish(msg);
}

}
