//
// Created by olivier on 9/29/16.
//

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

#include "QuadState.h"
#include "ImageProcessor.h"

#include "MessageHandler.h"

namespace localization {

MessageHandler::MessageHandler(const CameraInfo& cameraInfo, QuadState& state, ImageProcessor* processor)
    : it_(nh_), state_(state), processor_(processor), cameraInfo_(cameraInfo)
{
    imageSub_ = it_.subscribe(cameraInfo.topic, 1, &MessageHandler::cameraCallback, this);
}

MessageHandler::~MessageHandler()
{
}

void MessageHandler::lookForMessages()
{
    cv::VideoCapture vc;
    std::string videoPath = "/home/olstob/Videos/cam_bas.mp4";
    vc.open(videoPath.c_str());
    if (!vc.isOpened())
    {
        std::cerr << "Could not open " << videoPath << std::endl;
        //exit(-1);
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
    try {
        isWaitingForImage_ = true;
        ros::Time stamp = ros::Time::now();

        tfListener_.waitForTransform("elikos_arena_origin", "elikos_fcu",  stamp, ros::Duration(1.0));
        tfListener_.lookupTransform("elikos_arena_origin", "elikos_fcu", stamp, state_.origin2fcu);

        tfListener_.waitForTransform("elikos_fcu", cameraInfo_.frame, stamp, ros::Duration(1.0));
        tfListener_.lookupTransform("elikos_fcu", cameraInfo_.frame, stamp, state_.fcu2camera);
    }
    catch (tf::TransformException e) 
    {
        ROS_ERROR("%s", e.what());
    }

    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
    processor_->processImage(input, msg->header.stamp);
}

}