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
    std::string cam_name = ros::this_node::getName();
    cam_name = cam_name.substr(0, cam_name.size()-std::string("_detection").size());
    pub_ = nh_.advertise<elikos_msgs::RobotRawArray>("/"+cam_name+"/elikos_robot_raw_array", 1);
    pubCommandOutput_ = nh_.advertise<std_msgs::String>(RCCommandOutputTopic, 100);
    
    pubImages_ = it_.advertise(RCdebugTopic, 1);  // debug only
    pubRed_ = it_.advertise("camera/image_opencv_red", 1);//debug only
    pubGreen_ = it_.advertise("camera/image_opencv_green", 1);//debug only

    detection_.loadCalibration(calibrationFilename);

    // Calibration trackbars
    bool calibrationOn;
    if (nh_.getParam("/" + ros::this_node::getName() + "/calibration",calibrationOn)) {
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
                                     v_min, preErode, dilate, postErode, 6);
    } else if (command == "getCurrentState") {
        std_msgs::String msg;  // OutputCommand to be sent
        msg.data = "getCurrentState\t" + detection_.getAllParams();
        pubCommandOutput_.publish(msg);
    }
}

void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr& input) {
    cv::Mat currentImage = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;

    Mat threshold_w;
    Mat threshold_r;
    Mat threshold_g;
    Mat robotsMat;

    vector<RobotDesc> robotsArray;
    detection_.detect(currentImage, threshold_w, threshold_r, threshold_g, robotsMat, robotsArray);

    //debug images
    if (isCalibrating) {
        sensor_msgs::ImagePtr msgDebug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", robotsMat).toImageMsg();
        pubImages_.publish(msgDebug);

        sensor_msgs::ImagePtr msgDebug2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold_r).toImageMsg();
        pubRed_.publish(msgDebug2);

        sensor_msgs::ImagePtr msgDebug3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold_g).toImageMsg();
        pubGreen_.publish(msgDebug3);
    }

    // publishing data
    elikos_msgs::RobotRawArray output;
    elikos_msgs::RobotRaw data;

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





void MessageHandler::calibrate(const elikos_msgs::CalibDetection* const message)
{
    const elikos_msgs::ColorDetectionInfo* colors[3] = {&message->red, &message->green, &message->white};
    for(int i = 0; i < 3; ++i){
        detection_.fetchRemoteParams(
            i,
            colors[i]->col.max.h, 
            colors[i]->col.min.h, 
            colors[i]->col.max.s, 
            colors[i]->col.min.s, 
            colors[i]->col.max.v, 
            colors[i]->col.min.v, 
            colors[i]->deflate, 
            colors[i]->inflate, 
            colors[i]->postDeflate,
            colors[i]->blur
        );
    }
}

void MessageHandler::loadCalibration(const YAML::Node& fileContent)
{
    elikos_msgs::CalibDetection message;
    //valeurs par défaut
    message.red.col.max.h  = 170;
    message.red.col.min.h  = 10;
    message.red.col.max.s  = 170;
    message.red.col.min.s  = 10;
    message.red.col.max.v  = 170;
    message.red.col.min.v  = 10;
    message.red.deflate = 10;
    message.red.inflate = 170;
    message.red.postDeflate = 170;
    message.red.blur = 6;

    message.green.col.max.h  = 170;
    message.green.col.min.h  = 10;
    message.green.col.max.s  = 170;
    message.green.col.min.s  = 10;
    message.green.col.max.v  = 170;
    message.green.col.min.v  = 10;
    message.green.deflate = 10;
    message.green.inflate = 170;
    message.green.postDeflate = 170;
    message.green.blur = 6;

    message.white.col.max.h  = 170;
    message.white.col.min.h  = 10;
    message.white.col.max.s  = 170;
    message.white.col.min.s  = 10;
    message.white.col.max.v  = 170;
    message.white.col.min.v  = 10;
    message.white.deflate = 10;
    message.white.inflate = 170;
    message.white.postDeflate = 170;
    message.white.blur = 6;


    elikos_msgs::ColorDetectionInfo* colors[3] = {&message.red, &message.green, &message.white};
    std::string names[3] = {"red","green","white"};
    //chargement des valeurs
    for(int i = 0; i < 3; ++i){
        if(fileContent[ names[i]+"_max_h"])        colors[i]->col.max.h   = (uint8_t)fileContent[ names[i]+"_max_h"].as<int>();
        if(fileContent[ names[i]+"_min_h"])        colors[i]->col.min.h   = (uint8_t)fileContent[ names[i]+"_min_h"].as<int>();
        if(fileContent[ names[i]+"_max_s"])        colors[i]->col.max.s   = (uint8_t)fileContent[ names[i]+"_max_s"].as<int>();
        if(fileContent[ names[i]+"_min_s"])        colors[i]->col.min.s   = (uint8_t)fileContent[ names[i]+"_min_s"].as<int>();
        if(fileContent[ names[i]+"_max_v"])        colors[i]->col.max.v   = (uint8_t)fileContent[ names[i]+"_max_v"].as<int>();
        if(fileContent[ names[i]+"_min_v"])        colors[i]->col.min.v   = (uint8_t)fileContent[ names[i]+"_min_v"].as<int>();
        if(fileContent[ names[i]+"_deflate"])      colors[i]->deflate     = (uint8_t)fileContent[ names[i]+"_deflate"].as<int>();
        if(fileContent[ names[i]+"_inflate"])      colors[i]->inflate     = (uint8_t)fileContent[ names[i]+"_inflate"].as<int>();
        if(fileContent[ names[i]+"_post_deflate"]) colors[i]->postDeflate = (uint8_t)fileContent[ names[i]+"_post_deflate"].as<int>();
        if(fileContent[ names[i]+"_blur"])         colors[i]->blur        = (uint8_t)fileContent[ names[i]+"_blur"].as<int>();
    }

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
    int deflate;
    int inflate;
    int postDeflate;
    int blur;

    std::string colorNames[3] = {"red","green","white"};
    for (int i = 0; i < 3; ++i) {
        detection_.getRemoteParams(
            i,
            max_h,
            min_h,
            max_s,
            min_s,
            max_v,
            min_v,
            deflate,
            inflate,
            postDeflate,
            blur
        );

        fileContent[colorNames[i] + "_max_h"] = max_h;
        fileContent[colorNames[i] + "_min_h"] = min_h;
        fileContent[colorNames[i] + "_max_s"] = max_s;
        fileContent[colorNames[i] + "_min_s"] = min_s;
        fileContent[colorNames[i] + "_max_v"] = max_v;
        fileContent[colorNames[i] + "_min_v"] = min_v;
        fileContent[colorNames[i] + "_inflate"] = inflate;
        fileContent[colorNames[i] + "_deflate"] = deflate;
        fileContent[colorNames[i] + "_post_deflate"] = postDeflate;
        fileContent[colorNames[i] + "_blur"] = postDeflate;
    }
  
}
