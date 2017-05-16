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

MessageHandler::MessageHandler(string calibrationFilename) : it_(nh_) {
    std::string inputTopic, RCinputTopic, RCdebugTopic, RCCommandOutputTopic;

    if (!nh_.getParam("/" + ros::this_node::getName() + "/topic", inputTopic)) {
        inputTopic = "";
    }

    if (!nh_.getParam("/" + ros::this_node::getName() + "/RCpublishTopic",
                      RCinputTopic)) {
        RCinputTopic = "elikos_remotecalib_publishTopic";
    }

    if (!nh_.getParam("/" + ros::this_node::getName() + "/RClistenTopic",
                      RCdebugTopic)) {
        RCdebugTopic = "elikos_remotecalib_listenTopic";
    }
    if (!nh_.getParam(
            "/" + ros::this_node::getName() + "/CommandOutputListenTopic",
            RCCommandOutputTopic)) {
        RCCommandOutputTopic = "elikos_remotecalib_cmdOutputListenTopic";
    }
    is_ = it_.subscribe(inputTopic, 1, &MessageHandler::dispatchMessage, this);
    subRC_ = nh_.subscribe(RCinputTopic, 100, &MessageHandler::dispatchCommand,
                           this);
    std::string cam_name = ros::this_node::getName();
    cam_name = cam_name.substr(0, cam_name.size()-std::string("_detection").size());
    pub_ =
        nh_.advertise<elikos_ros::RobotRawArray>("/"+cam_name+"/elikos_robot_raw_array", 1);
    pubCommandOutput_ =
        nh_.advertise<std_msgs::String>(RCCommandOutputTopic, 100);
    // pubImages_ = it_.advertise(inputTopic + "/debug", 1); //debug only
    pubImages_ = it_.advertise(RCdebugTopic, 1);  // debug only
    // pubRed_ = it_.advertise("camera/image_opencv_red", 1);//debug only
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

    // debug images
    sensor_msgs::ImagePtr msgDebug =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", robotsMat).toImageMsg();
    pubImages_.publish(msgDebug);
    
    // sensor_msgs::ImagePtr msgDebug2 = cv_bridge::CvImage(std_msgs::Header(),
    // "bgr8", threshold_r).toImageMsg();
    // pubRed_.publish(msgDebug2);
    // sensor_msgs::ImagePtr msgDebug3 = cv_bridge::CvImage(std_msgs::Header(),
    // "bgr8", threshold_g).toImageMsg();
    // pubGreen_.publish(msgDebug3);

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
