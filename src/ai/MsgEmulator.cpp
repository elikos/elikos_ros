#include <thread>
#include <iostream>


#include "MsgEmulator.h"
#include <tf/transform_datatypes.h>

namespace ai 
{

const std::string MsgEmulator::WORLD_FRAME = "world";
const std::string MsgEmulator::TRGT_FRAME = "trgtRobot";
const std::string MsgEmulator::TOPIC = "target_robot_array";

const double MsgEmulator::virtualRadius = 10.0;
bool MsgEmulator::started_ = false;

MsgEmulator* MsgEmulator::instance_ = nullptr;

MsgEmulator::MsgEmulator()
{
    pub_ = nh_.advertise<elikos_ros::TargetRobotArray>(TOPIC, 1);
}

bool MsgEmulator::start()
{
    if (!started_)
    {
        std::thread th(&MsgEmulator::lookForTf, this);
        th.detach();
        started_ = true;
        return true;
    }
    return false;
}

MsgEmulator* MsgEmulator::getInstance()
{
   if (instance_ == nullptr)
   {
       instance_ = new MsgEmulator();
   }
   return instance_;
}

void MsgEmulator::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void MsgEmulator::lookForTf()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        lookForTargets();
        ros::spinOnce();
        rate.sleep();
    }
}

void MsgEmulator::lookForTargets()
{
    for (int i = 0; i < N_TRGT; i++)
    {
        tf::StampedTransform stf;
        try
        {
            listener_.lookupTransform(WORLD_FRAME, TRGT_FRAME + std::to_string(i), ros::Time(0), stf);
            addTarget(stf, i);
        }
        catch(tf::TransformException e)
        {
            ROS_ERROR("%s", e.what());
            continue;
        }
    }
    pub_.publish(targets_);
}

void MsgEmulator::addTarget(const tf::StampedTransform& stf, unsigned char id)
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
