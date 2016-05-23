#include "MessageHandler.h"

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "BlobDetection/BlobTracking.h"

namespace dnt
{

MessageHandler::MessageHandler(string calibrationFilename) : 
    it_(nh_)
{
	is_ = it_.subscribe("camera/image_raw", 1, &MessageHandler::dispatchMessage, this);
    pub_ = nh_.advertise<elikos_ros::RobotRawArray>("robot_raw_array", 1);
    pubImages_ = it_.advertise("camera/image_opencv", 1);//debug only
    
	tracking_.loadCalibration(calibrationFilename);
	
    //Calibration trackbars
	bool calibrationOn;
    if (nh_.getParam("/dnt/calibration", calibrationOn))
	{
		if(calibrationOn) tracking_.createTrackbars();
	}
}


MessageHandler::~MessageHandler()
{
}


void MessageHandler::dispatchMessage(const sensor_msgs::ImageConstPtr &input)
{
    cv::Mat currentImage = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image;

    Mat threshold_w;
    Mat threshold_r;
    Mat threshold_g;
    Mat robotsMat;

    tracking_.track(currentImage, threshold_w, threshold_r, threshold_g, robotsMat);
	//debug image
	sensor_msgs::ImagePtr msgDebug = cv_bridge::CvImage(std_msgs::Header(), "bgr8", robotsMat).toImageMsg();
	pubImages_.publish(msgDebug);
	
    //publishing data
    elikos_ros::RobotRawArray output;
    elikos_ros::RobotRaw data;

    for(auto robot : tracking_.getRobots()){
		data.id = robot.getID();
		data.color = robot.getColor();
		data.pose.x = robot.getXPos();
		data.pose.y = robot.getYPos();
		data.pose.theta = robot.getDirection();
		output.robots.push_back(data);
		pub_.publish(output);
    }
}

void MessageHandler::saveCalibration(string filename){
	tracking_.saveCalibration(filename);
}
}
