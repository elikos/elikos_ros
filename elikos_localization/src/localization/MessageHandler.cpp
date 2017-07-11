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
    imagePub_ = it_.advertise(cameraInfo.name + "/intersection_detection", 1);
}

MessageHandler::~MessageHandler()
{
}

void MessageHandler::lookForMessages()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (state_.update(msg->header.stamp))
    {
        cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;

        cv::Mat result;
        processor_->processImage(input, result);

        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
        imagePub_.publish(image);
    }
}

}