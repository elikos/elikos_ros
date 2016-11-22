#ifndef MESSAGE_HANDLER_REMOTECALIB
#define MESSAGE_HANDLER_REMOTECALIB

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include "gui/headers/CalibrationWindow.hpp"
#include "gui/headers/ControlWindow.hpp"

class MessageHandler
{
public:
  MessageHandler(CalibrationWindow &calibWindow, ControlWindow &controlWindow);
  ~MessageHandler();
  void dispatchMessage(const sensor_msgs::ImageConstPtr &input);
  void dispatchDebugImage(const sensor_msgs::ImageConstPtr &input);
  void dispatchCommandOutput(const std_msgs::String::ConstPtr &input);

  void saveCalibration();
  void sendRefreshCommand() const
  {
    /* INITIALISATION DES TRACKBARS */
    std_msgs::String initMsg;
    initMsg.data = "getCurrentState";
    pub_.publish(initMsg);
  }

  const std::string &IO_IMG_ENCODING_TYPE = sensor_msgs::image_encodings::BGR8;

private:
  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport imgTransport_;

  image_transport::Subscriber imgSubscriber_;
  image_transport::Subscriber debugimgSubscriber_;

  ros::Publisher pub_;
  ros::Subscriber subCommandResult_;

  CalibrationWindow &calibWindow_;
  ControlWindow &controlWindow_;

};

#endif /// MESSAGE_HANDLER_REMOTECALIB
