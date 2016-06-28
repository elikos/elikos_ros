#ifndef AI_MESSAGE_EMULATOR_H
#define AI_MESSAGE_EMULATOR_H

#include <elikos_ros/TargetRobot.h>

#include <elikos_ros/TargetRobotArray.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>


namespace ai
{

class MessageEmulator
{
public:
    static const int N_TRGT = 10;
    static const double virtualRadius;

    static const std::string TRGT_FRAME;
    static const std::string MAV_FRAME;
    static const std::string TOPIC;
    static const std::string WORLD_FRAME;

    static MessageEmulator* getInstance();
    static void freeInstance();

    void lookForTf();
    bool start();

private:
    static MessageEmulator* instance_;

    static bool started_;

    elikos_ros::TargetRobotArray targets_;

    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    ros::Publisher mavPub_;
    ros::Publisher trgtPub_;

    MessageEmulator();
    ~MessageEmulator() = default;
    void lookForTargets();
    void lookForMav();
    void addTarget(const tf::StampedTransform& stf, unsigned char i);
};

}

#endif /// AI_MESSAGE_EMULATOR_H
