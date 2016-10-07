#include "MessageHandler.h"

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

MessageHandler::MessageHandler(string calibrationFilename) : it_(nh_)
{
  std::string inputTopic;
  if (!nh_.getParam("/" + ros::this_node::getName() + "/topic", inputTopic))
  {
    inputTopic = "";
  }

  is_ = it_.subscribe(inputTopic, 1, &MessageHandler::dispatchMessage, this);

  pub_ = nh_.advertise<elikos_ros::RobotRawArray>("elikos_robot_raw_array", 1);
  pubImages_ = it_.advertise(inputTopic + "/debug", 1); //debug only
                                                        //pubRed_ = it_.advertise("camera/image_opencv_red", 1);//debug only
                                                        //pubGreen_ = it_.advertise("camera/image_opencv_green", 1);//debug only

  detection_.loadCalibration(calibrationFilename);

  //Calibration trackbars
  bool calibrationOn;
  if (nh_.getParam("/" + ros::this_node::getName() + "/calibration", calibrationOn))
  {
    if (calibrationOn)
      detection_.createTrackbars();
  }
}

MessageHandler::~MessageHandler()
{
}

// void MessageHandler::rangeMessage(const sensor_msgs::Range::ConstPtr &input)
// {
//   ROS_INFO("GOT IT");
//   std::cout<<"DONE"<<endl;
// }

void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr &input)
{
  cv::Mat currentImage = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;

  Mat threshold_w;
  Mat threshold_r;
  Mat threshold_g;
  Mat robotsMat;

  detection_.detect(currentImage, threshold_w, threshold_r, threshold_g, robotsMat);

  //debug images
  sensor_msgs::ImagePtr msgDebug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", robotsMat).toImageMsg();
  pubImages_.publish(msgDebug);
  //sensor_msgs::ImagePtr msgDebug2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", threshold_r).toImageMsg();
  //pubRed_.publish(msgDebug2);
  //sensor_msgs::ImagePtr msgDebug3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", threshold_g).toImageMsg();
  //pubGreen_.publish(msgDebug3);

  //publishing data
  elikos_ros::RobotRawArray output;
  elikos_ros::RobotRaw data;

  for (auto robot : detection_.getRobots())
  {
    data.id = robot.getID();
    data.color = robot.getColor();
    data.point.x = robot.getXPos();
    data.point.y = robot.getYPos();
    data.point.z = 0;
    output.robots.push_back(data);
    pub_.publish(output);
  }
}

void MessageHandler::saveCalibration(string filename)
{
  detection_.saveCalibration(filename);
}
