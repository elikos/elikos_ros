#include "Translator.h"

namespace ai 
{
 

const std::string Translator::WORLD_FRAME = "world";
const std::string Translator::TRGT_FRAME = "trgt_frame";
const std::string Translator::TOPIC = "target_robot_array";

Translator* Translator::instance_ = nullptr;

Translator::Translator()
{
}

Translator* Translator::getInstance()
{
   if (instance_ == nullptr)
   {
       instance_ = new Translator();
   }
   return instance_;
}

void Translator::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

void Translator::lookForTf()
{
    ros::Rate rate(30);
    while(ros::ok()) 
    {
        lookForTargets();
        ros::spinOnce();
        rate.sleep();
    }
}


void Translator::lookForTargets()
{
    for (int i = 0; i < N_TRGT; i++)
    {
        tf::StampedTransform stf;
        try
        {
            listener_.lookupTransform(WORLD_FRAME, TRGT_FRAME + std::to_string(i), ros::Time(0), stf);
        }
        catch(tf::TransformException e)
        {
            ROS_ERROR("%s", e.what());
            continue;
        }
    }
}
    
};
