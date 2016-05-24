#ifndef AI_TRANSLATOR
#define AI_TRANSLATOR

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace ai
{

class Translator
{
public:
    static const int N_TRGT = 10;

    static const std::string WORLD_FRAME;
    static const std::string TRGT_FRAME;

    static const std::string TOPIC;

    static Translator* getInstance();
    static void freeInstance();

    void lookForTf();

private:
    ~Translator() = default;
    Translator();

    static Translator* instance_;

    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    ros::Publisher pubisher_;

    void lookForTargets();
};

}

#endif /// AI_TRANSLATER
