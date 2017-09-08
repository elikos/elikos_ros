//
// Created by olivier on 9/29/16.
//

#include "MessageHandler.h"
#include <cv_bridge/cv_bridge.h>

namespace preprocessing {

MessageHandler::MessageHandler(const ros::NodeHandle& nodeHandle, const std::string& cameraName)
    : nh_(nodeHandle), it_(nh_), preProcessing_(cameraInfo_, state_)
{
    if (cameraInfo_.load(cameraName)) {
        state_.setCameraFrame(cameraInfo_.frame);
        imageSub_ = it_.subscribeCamera(
                cameraInfo_.topic,
                1,
                &MessageHandler::cameraCallback,
                this
        );
        preprocessedPub_ = it_.advertise(cameraInfo_.frame + "/image_preprocessed", 1);
        inverseTransformPub_ = nh_.advertise<elikos_ros::StampedMatrix3>("image_preprocessed/inverse_transform", 1);
    } else {
        ROS_ERROR("Failed to load config file.");
    }

}

MessageHandler::~MessageHandler()
{
}

void MessageHandler::cameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    state_.update(image_msg->header.stamp);

    cv::Mat input = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat output;
    cv::Mat inverseTransform;

    //TODO avoir des longeurs focales différentes pour x et y
    preProcessing_.preProcessImage(input, output, inverseTransform);

    sensor_msgs::ImagePtr msgPreproc = cv_bridge::CvImage(image_msg->header,
                                                          sensor_msgs::image_encodings::BGR8,
                                                          output).toImageMsg();
    preprocessedPub_.publish(msgPreproc);

    elikos_ros::StampedMatrix3 invTransformMsg;
    invTransformMsg.header = image_msg->header;
    for(int i = 0; i < 9; ++i){
        invTransformMsg.matrix.data[i] = inverseTransform.at<double>(i/3, i%3);
    }
    inverseTransformPub_.publish(invTransformMsg);
}

}