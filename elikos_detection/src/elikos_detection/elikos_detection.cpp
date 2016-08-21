#include <memory>
#include <ros/ros.h>
#include <ros/package.h>
#include "MessageHandler.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_detection" );

  	string extension = ".calib";
  	string inputFile, outputFile;

    ros::NodeHandle nh_;

	  string calibFolderPath;
    if (!nh_.getParam("/elikos_detection/dir", calibFolderPath))
  	{
  		calibFolderPath = "";
  	}
    if (!nh_.getParam("/elikos_detection/file_in", inputFile))
  	{
  		inputFile = "calibration_initial";
  	}
    if (!nh_.getParam("/elikos_detection/file_out", outputFile))
  	{
  		outputFile = "new_calibration";
  	}
    MessageHandler messageHandler(calibFolderPath+inputFile+extension);

    bool calibrationOn;
    nh_.getParam("/elikos_detection/calibration", calibrationOn);

    ros::Rate r(30);
    // Endless loop
    while(ros::ok())
    {
        ros::spinOnce();
        if(calibrationOn) {
    			//if the 's' key is pressed, that save the calibration.
    			int key = waitKey(20);
    			if(((char)key)=='s') messageHandler.saveCalibration(calibFolderPath+outputFile+extension);
    		}
        r.sleep();
    }
}
