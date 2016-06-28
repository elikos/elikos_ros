#include <thread>

#include <iostream>

#include "MessageEmulator.h"

#include "MessageHandler.h"

namespace ai
{

const std::string MessageEmulator::WORLD_FRAME = "world";
const std::string MessageEmulator::MAV_FRAME = "MAV";
const std::string MessageEmulator::TRGT_FRAME = "trgtRobot";

const double MessageEmulator::virtualRadius = 10.0;
bool MessageEmulator::started_ = false;

MessageEmulator* MessageEmulator::instance_ = nullptr;

MessageEmulator::MessageEmulator()
{
    trgtPub_ = nh_.advertise<elikos_ros::TargetRobotArray>(MessageHandler::TRGT_TOPIC, 1);
    mavPub_ = nh_.advertise<geometry_msgs::PoseStamped>(MessageHandler::MAV_TOPIC, 1);
}

bool MessageEmulator::start()
{
    if (!started_)
    {
        std::thread th(&MessageEmulator::lookForTf, this);
        th.detach();
        started_ = true;
        return true;
    }
    return false;
}

MessageEmulator* MessageEmulator::getInstance()
{
   if (instance_ == nullptr)
   {
       instance_ = new MessageEmulator();
   }
   return instance_;
}

void MessageEmulator::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MessageEmulator::lookForTf()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        lookForTargets();
        lookForMav();
        ros::spinOnce();
        rate.sleep();
    }
}

void MessageEmulator::lookForMav()
{
    tf::StampedTransform stf;
    try
    {
        listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), stf);
    }
    catch (tf::TransformException e)
    {
        return;
    }

    // From stamped transform to stamped pose
    tf::Stamped<tf::Pose> poseTf;
    poseTf.setOrigin(stf.getOrigin());
    poseTf.setRotation(stf.getRotation());

    // From tf to msg
    geometry_msgs::PoseStamped poseMsg;
    tf::poseStampedTFToMsg(poseTf, poseMsg);
    mavPub_.publish(poseMsg);
}


void MessageEmulator::lookForTargets()
{
    targets_.targets.clear();
    assert(targets_.targets.size() == 0);
    for (int i = 0; i < N_TRGT; ++i)
    {
        tf::StampedTransform stf;
        try
        {
            listener_.lookupTransform(WORLD_FRAME, TRGT_FRAME + std::to_string(i), ros::Time(0), stf);
            addTarget(stf, i);
        }
        catch(tf::TransformException e)
        {
            continue;
        }
    }
    trgtPub_.publish(targets_);
}

void MessageEmulator::addTarget(const tf::StampedTransform& stf, unsigned char id)
{
    tf::Pose poseTf(stf);
    geometry_msgs::Pose poseMsg;
    tf::poseTFToMsg(poseTf, poseMsg);

    elikos_ros::TargetRobot target;
    target.color = 0;
    target.id = id;
    target.poseOrigin.pose = poseMsg;
    targets_.targets.push_back(target);
}
    
};
