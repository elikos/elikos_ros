#include <memory>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "MessageHandler.h"

namespace preprocessing{
class ElikosPreprocessing : public nodelet::Nodelet{
public:
    virtual void onInit(){
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        MessageHandler* pointer = new MessageHandler(nh, pnh);
        messageHandler_ = std::unique_ptr<MessageHandler>(pointer);
    } 
private:
    std::unique_ptr<MessageHandler> messageHandler_;
};
}

/** pas de main dans un nodelet*/

//Exportation de la classe principale du nodelet
PLUGINLIB_EXPORT_CLASS(preprocessing::ElikosPreprocessing, nodelet::Nodelet)


