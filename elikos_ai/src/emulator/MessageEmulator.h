#ifndef AI_MESSAGE_EMULATOR_H
#define AI_MESSAGE_EMULATOR_H

#include <elikos_ros/TargetRobot.h>

#include <elikos_ros/TargetRobotArray.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


namespace emu
{

class MessageEmulator
{
public:
    static const int N_TRGT = 10;

    static const std::string SIM_TRGT_FRAME;
    static const std::string SIM_MAV_FRAME;
    static const std::string SIM_DST_TOPIC;



    static MessageEmulator* getInstance();
    static void freeInstance();

    void lookForTf();

private:
    static MessageEmulator* instance_;

    elikos_ros::TargetRobotArray targets_;

    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
    ros::Publisher trgtPub_;
    ros::Publisher dstPub_;
    ros::Subscriber dstSub_;

    tf::Point mavPosition_;

    MessageEmulator();
    ~MessageEmulator() = default;
    void lookForTargets();
    void lookForMav();
    void addTarget(const tf::StampedTransform& stf, unsigned char i);
    void handleDstMsg(geometry_msgs::PoseStamped input);
};

}

#endif /// AI_MESSAGE_EMULATOR_H
