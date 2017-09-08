#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "CameraInfo.h"
#include "QuadState.h"

#include "MessageHandler.h"

namespace preprocessing {

class ElikosPreprocessing : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        std::vector<std::string> args = getMyArgv();
        if (args.size() > 0)
        {
            messageHandler_ = std::unique_ptr<MessageHandler>(new MessageHandler(getNodeHandle(), args[0]));
        }
        else
        {
            ROS_ERROR("Expected at least one argument");
        }
    }
private:
    std::unique_ptr<MessageHandler> messageHandler_;
};
}

/** pas de main dans un nodelet*/

//Exportation de la classe principale du nodelet
PLUGINLIB_EXPORT_CLASS(preprocessing::ElikosPreprocessing, nodelet::Nodelet)


