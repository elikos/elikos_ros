#ifndef AI_TRANSLATOR
#define AI_TRANSLATOR

#include <elikos_ros/TargetRobot.h>
#include <elikos_ros/TargetRobotArray.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace ai
{

class MsgEmulator
{
public:
    static const int N_TRGT = 10;
    static const double virtualRadius;

    static const std::string WORLD_FRAME;
    static const std::string TRGT_FRAME;
    static const std::string TOPIC;

    static MsgEmulator* getInstance();
    static void freeInstance();

    void lookForTf();
    bool start();

private:
    static MsgEmulator* instance_;

    static bool started_;

    elikos_ros::TargetRobotArray targets_;

    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    ros::Publisher pub_;

    MsgEmulator();
    ~MsgEmulator() = default;
    void lookForTargets();
    void addTarget(const tf::StampedTransform& stf, unsigned char i);
};

}

#endif /// AI_TRANSLATER
