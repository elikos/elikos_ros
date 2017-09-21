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
        std::string cameraName;
        if (getPrivateNodeHandle().getParam("camera_name", cameraName))
        {
            messageHandler_ = new MessageHandler(getPrivateNodeHandle(), cameraName);
        }
        else
        {
            ROS_ERROR("Expected camera_name parameter not found");
        }
    }
private:
   MessageHandler* messageHandler_;
};

}

//Exportation de la classe principale du nodelet
PLUGINLIB_EXPORT_CLASS(preprocessing::ElikosPreprocessing, nodelet::Nodelet)


