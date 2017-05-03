#include "MessageHandler.h"

#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cassert>


MessageHandler::MessageHandler(string calibrationFilename) 
    : nh_("~")
    , it_(nh_)
{
    std::string inputTopic, RCinputTopic, RCdebugTopic, RCCommandOutputTopic;

    if (!nh_.getParam("/" + ros::this_node::getName() + "/topic", inputTopic)) {
        inputTopic = "";
    }

    if (!nh_.getParam("/" + ros::this_node::getName() + "/RCpublishTopic", RCinputTopic)) {
        RCinputTopic = "elikos_remotecalib_publishTopic";
    }

    if (!nh_.getParam("/" + ros::this_node::getName() + "/RClistenTopic", RCdebugTopic)) {
        RCdebugTopic = "elikos_remotecalib_listenTopic";
    }
    if (!nh_.getParam("/" + ros::this_node::getName() + "/CommandOutputListenTopic", RCCommandOutputTopic)) {
        RCCommandOutputTopic = "elikos_remotecalib_cmdOutputListenTopic";
    }
    
    is_ = it_.subscribe(inputTopic, 1, &MessageHandler::dispatchMessage, this);
    subRC_ = nh_.subscribe(RCinputTopic, 100, &MessageHandler::dispatchCommand, this);
    pub_ = nh_.advertise<elikos_ros::RobotRawArray>("elikos_robot_raw_array", 1);

    pubCommandOutput_ = nh_.advertise<std_msgs::String>(RCCommandOutputTopic, 100);
    // pubImages_ = it_.advertise(inputTopic + "/debug", 1); //debug only
    pubImages_ = it_.advertise(RCdebugTopic, 1);  // debug only
    pubRed_ = it_.advertise("camera/image_opencv_red", 1);//debug only
    // pubGreen_ = it_.advertise("camera/image_opencv_green", 1);//debug only

    detection_.loadCalibration(calibrationFilename);

    // Calibration trackbars
    bool calibrationOn;
    if (nh_.getParam("/" + ros::this_node::getName() + "/calibration",
                     calibrationOn)) {
        if (calibrationOn) detection_.createTrackbars();
    }
}

MessageHandler::~MessageHandler() {}

void MessageHandler::dispatchCommand(const std_msgs::String::ConstPtr& input) {
    std::string msg = input->data;
    std::istringstream iss(msg);
    // Recoit "saveCalib" ou "update \t H \t S \t V \t DELTA" ou "updHSV (...)"
    std::string command;
    iss >> command;

    if (command == "saveCalib") {
        string calibFolderPath, outputFile;
        if (!nh_.getParam("/" + ros::this_node::getName() + "/dir",
                          calibFolderPath)) {
            calibFolderPath = "";
        }
        if (!nh_.getParam("/" + ros::this_node::getName() + "/file_out",
                          outputFile)) {
            outputFile = "new_calibration";
        }
        this->saveCalibration(calibFolderPath + "/" + outputFile + ".calib");
    } else if (command == "update") {
        int color, h, s, v, delta;
        iss >> color >> h >> s >> v >> delta;
        detection_.updateHSV(color, h, s, v, delta);
    } else if (command == "updHSV")  // Met a jour le HSV et les pre_erode,
                                     // dilate et post_erode
    {
        int color, h_max, h_min, s_max, s_min, v_max, v_min, preErode, dilate,
            postErode;
        iss >> color >> h_max >> h_min >> s_max >> s_min >> v_max >> v_min >>
            preErode >> dilate >> postErode;
        detection_.fetchRemoteParams(color, h_max, h_min, s_max, s_min, v_max,
                                     v_min, preErode, dilate, postErode);
    } else if (command == "getCurrentState") {
        std_msgs::String msg;  // OutputCommand to be sent
        msg.data = "getCurrentState\t" + detection_.getAllParams();
        pubCommandOutput_.publish(msg);
    }
}

void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr& input) {
    cv::Mat currentImage =
        cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;

    Mat threshold_w;
    Mat threshold_r;
    Mat threshold_g;
    Mat robotsMat;

    vector<RobotDesc> robotsArray;
    detection_.detect(currentImage, threshold_w, threshold_r, threshold_g,
                      robotsMat, robotsArray);

    //debug images
    if (isCalibrating) {
        std::cerr << "sending" << std::endl;
        sensor_msgs::ImagePtr msgDebug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", robotsMat).toImageMsg();
        pubImages_.publish(msgDebug);

        sensor_msgs::ImagePtr msgDebug2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold_r).toImageMsg();
        pubRed_.publish(msgDebug2);

        //sensor_msgs::ImagePtr msgDebug3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", threshold_g).toImageMsg();
        //pubGreen_.publish(msgDebug3);
    }
        std::cerr << "!sending" << std::endl;

    // publishing data
    elikos_ros::RobotRawArray output;
    elikos_ros::RobotRaw data;

    for (auto robot : robotsArray) {
        data.id = robot.getID();
        data.color = robot.getColor();
        data.point.x = robot.getXPos();
        data.point.y = robot.getYPos();
        data.point.z = 0;
        output.robots.push_back(data);
    }
    pub_.publish(output);
}

void MessageHandler::saveCalibration(string filename) {
    detection_.saveCalibration(filename);
}





void MessageHandler::calibrate(const elikos_remote_calib_client::CalibDetection* const message)
{
  detection_.fetchRemoteParams(
    0,
    message->red.max.h, 
    message->red.min.h, 
    message->red.max.s, 
    message->red.min.s, 
    message->red.max.v, 
    message->red.min.v, 
    message->redInflate, 
    message->redDeflate, 
    message->redInflate);//FIXME ON A BESION D'UN AUTRE PARAMÈTRE
  detection_.fetchRemoteParams(
    1,
    message->green.max.h, 
    message->green.min.h, 
    message->green.max.s, 
    message->green.min.s, 
    message->green.max.v, 
    message->green.min.v, 
    message->greenInflate, 
    message->greenDeflate, 
    message->greenInflate);//FIXME ON A BESION D'UN AUTRE PARAMÈTRE
  detection_.fetchRemoteParams(
    2,
    message->white.max.h, 
    message->white.min.h, 
    message->white.max.s, 
    message->white.min.s, 
    message->white.max.v, 
    message->white.min.v, 
    message->whiteInflate, 
    message->whiteDeflate, 
    message->whiteInflate);//FIXME ON A BESION D'UN AUTRE PARAMÈTRE
}

void MessageHandler::loadCalibration(const YAML::Node& fileContent)
{
  elikos_remote_calib_client::CalibDetection message;
  //valeurs par défaut
  message.red.max.h  = 170;
  message.red.min.h  = 10;
  message.red.max.s  = 170;
  message.red.min.s  = 10;
  message.red.max.v  = 170;
  message.red.min.v  = 10;
  message.redInflate = 170;
  message.redDeflate = 10;

  message.green.max.h  = 170;
  message.green.min.h  = 10;
  message.green.max.s  = 170;
  message.green.min.s  = 10;
  message.green.max.v  = 170;
  message.green.min.v  = 10;
  message.greenInflate = 170;
  message.greenDeflate = 10;

  message.white.max.h  = 170;
  message.white.min.h  = 10;
  message.white.max.s  = 170;
  message.white.min.s  = 10;
  message.white.max.v  = 170;
  message.white.min.v  = 10;
  message.whiteInflate = 170;
  message.whiteDeflate = 10;

  //chargement des valeurs
  if(fileContent["red_max_h"])  message.red.max.h  = fileContent["red_max_h"].as<uint8_t>();
  if(fileContent["red_min_h"])  message.red.min.h  = fileContent["red_min_h"].as<uint8_t>();
  if(fileContent["red_max_s"])  message.red.max.s  = fileContent["red_max_s"].as<uint8_t>();
  if(fileContent["red_min_s"])  message.red.min.s  = fileContent["red_min_s"].as<uint8_t>();
  if(fileContent["red_max_v"])  message.red.max.v  = fileContent["red_max_v"].as<uint8_t>();
  if(fileContent["red_min_v"])  message.red.min.v  = fileContent["red_min_v"].as<uint8_t>();
  if(fileContent["red_inflate"])message.redInflate = fileContent["red_inflate"].as<uint8_t>();
  if(fileContent["red_deflate"])message.redDeflate = fileContent["red_deflate"].as<uint8_t>();

  if(fileContent["green_max_h"])  message.green.max.h  = fileContent["green_max_h"].as<uint8_t>();
  if(fileContent["green_min_h"])  message.green.min.h  = fileContent["green_min_h"].as<uint8_t>();
  if(fileContent["green_max_s"])  message.green.max.s  = fileContent["green_max_s"].as<uint8_t>();
  if(fileContent["green_min_s"])  message.green.min.s  = fileContent["green_min_s"].as<uint8_t>();
  if(fileContent["green_max_v"])  message.green.max.v  = fileContent["green_max_v"].as<uint8_t>();
  if(fileContent["green_min_v"])  message.green.min.v  = fileContent["green_min_v"].as<uint8_t>();
  if(fileContent["green_inflate"])message.greenInflate = fileContent["green_inflate"].as<uint8_t>();
  if(fileContent["green_deflate"])message.greenDeflate = fileContent["green_deflate"].as<uint8_t>();

  if(fileContent["white_max_h"])  message.white.max.h  = fileContent["white_max_h"].as<uint8_t>();
  if(fileContent["white_min_h"])  message.white.min.h  = fileContent["white_min_h"].as<uint8_t>();
  if(fileContent["white_max_s"])  message.white.max.s  = fileContent["white_max_s"].as<uint8_t>();
  if(fileContent["white_min_s"])  message.white.min.s  = fileContent["white_min_s"].as<uint8_t>();
  if(fileContent["white_max_v"])  message.white.max.v  = fileContent["white_max_v"].as<uint8_t>();
  if(fileContent["white_min_v"])  message.white.min.v  = fileContent["white_min_v"].as<uint8_t>();
  if(fileContent["white_inflate"])message.whiteInflate = fileContent["white_inflate"].as<uint8_t>();
  if(fileContent["white_deflate"])message.whiteDeflate = fileContent["white_deflate"].as<uint8_t>();

  //fire du event
  loadRemoteCalibration(message);
}


void MessageHandler::saveCalibration(YAML::Node& fileContent)
{
  int max_h;
  int min_h;
  int max_s;
  int min_s;
  int max_v;
  int min_v;
  int inflate;
  int deflate;
  int postInflate;

  detection_.getRemoteParams(
    0,
    max_h,
    min_h,
    max_s,
    min_s,
    max_v,
    min_v,
    inflate,
    deflate,
    postInflate
  );

  fileContent["red_max_h"] = max_h;
  fileContent["red_min_h"] = min_h;
  fileContent["red_max_s"] = max_s;
  fileContent["red_min_s"] = min_s;
  fileContent["red_max_v"] = max_v;
  fileContent["red_min_v"] = min_v;
  fileContent["red_inflate"] = inflate;
  fileContent["red_deflate"] = deflate;

  detection_.getRemoteParams(
    1,
    max_h,
    min_h,
    max_s,
    min_s,
    max_v,
    min_v,
    inflate,
    deflate,
    postInflate
  );

  fileContent["green_max_h"] = max_h;
  fileContent["green_min_h"] = min_h;
  fileContent["green_max_s"] = max_s;
  fileContent["green_min_s"] = min_s;
  fileContent["green_max_v"] = max_v;
  fileContent["green_min_v"] = min_v;
  fileContent["green_inflate"] = inflate;
  fileContent["green_deflate"] = deflate;

  detection_.getRemoteParams(
    2,
    max_h,
    min_h,
    max_s,
    min_s,
    max_v,
    min_v,
    inflate,
    deflate,
    postInflate
  );

  fileContent["green_max_h"] = max_h;
  fileContent["green_min_h"] = min_h;
  fileContent["green_max_s"] = max_s;
  fileContent["green_min_s"] = min_s;
  fileContent["green_max_v"] = max_v;
  fileContent["green_min_v"] = min_v;
  fileContent["green_inflate"] = inflate;
  fileContent["green_deflate"] = deflate;
}
