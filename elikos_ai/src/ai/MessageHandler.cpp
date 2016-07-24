#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include "Agent.h"

#include "MessageHandler.h"

namespace ai
{

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
        lookForMav();
        agent_->behave();
        rate.sleep();
    }
}

void MessageHandler::handleTrgtMsg(const elikos_ros::TargetRobotArray::ConstPtr& input)
{
    agent_->updateTargets(input);
}

void MessageHandler::lookForMav()
{
    tf::StampedTransform stf;

    try {
        mavListener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), stf);
    } catch(tf::TransformException e) {
        return;
    }

    // From stamped transform to stamped pose
    tf::Stamped<tf::Pose> poseTf;
    poseTf.setOrigin(stf.getOrigin());
    poseTf.setRotation(stf.getRotation());

    Agent::getInstance()->updateQuadRobot(poseTf);
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
