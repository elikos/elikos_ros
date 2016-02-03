#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


void cameraCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
}

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_dnt" );
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Loop Rate : 30Hz
    ros::Rate r(30);

    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_Raw", 1, cameraCallback);

    // Endless loop
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
