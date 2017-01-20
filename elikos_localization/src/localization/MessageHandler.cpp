//
// Created by olivier on 9/29/16.
//

#include "MessageHandler.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

#include "ImageProcessor.h"

namespace localization {

MessageHandler* MessageHandler::instance_ = nullptr;

MessageHandler::MessageHandler()
    : it_(nh_)
{
    imageSub_ = it_.subscribe("/camera/image_raw", 1, &MessageHandler::cameraCallback, this);
    imuSub_ = nh_.subscribe("/mavros/imu/data", 1, &MessageHandler::imuCallback, this);
    poseSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &MessageHandler::poseCallback, this);
}

MessageHandler::~MessageHandler()
{
}

MessageHandler* MessageHandler::getInstance()
{
    if (instance_ == nullptr) {
        instance_ = new MessageHandler();
    }
    return instance_;
}

void MessageHandler::freeInstance()
{
    delete instance_;
}

void MessageHandler::lookForMessages()
{
    cv::VideoCapture vc;
    std::string videoPath = "/home/olivier/Videos/cam_bas.mp4";
    vc.open(videoPath.c_str());
    if (!vc.isOpened())
    {
        std::cerr << "Could not open " << videoPath << std::endl;
        exit(-1);
    }
    ros::Rate rate(30);
    while(ros::ok())
    {
        //cv::Mat frame;
        //vc >> frame;
        //ImageProcessor::getInstance()->processImage(frame, ros::Time::now());

        ros::spinOnce();
        rate.sleep();
    }
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
    ImageProcessor::getInstance()->processImage(input, msg->header.stamp);
}

void MessageHandler::imuCallback(const sensor_msgs::Imu& msg)
{
    tf::Vector3 v;
    tf::vector3MsgToTF(msg.linear_acceleration, v);
    v.normalize();
    ImageProcessor::getInstance()->imuOrientation_ = { v.x(), v.y(), v.z() };
}

void MessageHandler::poseCallback(const geometry_msgs::PoseStamped& msg)
{
    tf::Vector3 v(1.0, 0.0, 0.0);
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ImageProcessor::getInstance()->roll_ = roll;
    ImageProcessor::getInstance()->pitch_ = pitch;
}

}