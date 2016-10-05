#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ArenaTracking.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
    // ROS Init
    ros::init(argc, argv, "IARC7_Localization_Node");
    //ros::NodeHandlePtr nh(new ros::NodeHandle);
    ros::NodeHandle nh;
    ros::Rate rate(30);

    // Main feature detection class
    ArenaTracking arenaTracking(&nh);

    // Display windows
    namedWindow("input_", CV_WINDOW_AUTOSIZE);
    // cvSetWindowProperty("input_", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    namedWindow("output_",1);

    while(ros::ok()) {
        // Poll new messages
        ros::spinOnce();

        if(arenaTracking.isInitialized() && !arenaTracking.GetInputFrame().empty()) {
            // Display output
            imshow("input_", arenaTracking.GetInputFrame());
            imshow("output_", arenaTracking.GetProcessedFrame());
            waitKey(1);
        }
        // Sleep for the rest of the iteration
        rate.sleep();
    }
    return 0;
}