#include <memory>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <elikos_remote_calib_client/Calibrator.h>
#include <elikos_remote_calib_client/Calibratable.h>
#include <elikos_remote_calib_client/CalibPreprocessing.h>
#include "MessageHandler.h"

class Tmp : public remote_calib::Calibratable<elikos_remote_calib_client::CalibPreprocessing>{
  virtual void calibrate(const elikos_remote_calib_client::CalibPreprocessing* const message)
  {
    std::cout << message << std::endl;
  }

  virtual void loadCalibration(const YAML::Node& fileContent)
  {
    std::cerr << "Loading calibration !" << std::endl;
    std::cerr << fileContent["name"] << std::endl;
  }

  virtual void saveCalibration(YAML::Node& fileContent)
  {
    fileContent["encoding"] = "CHTAFF";
    fileContent["version"] = "6.6.7";
    fileContent["name"] = "Outaf Hell";
  }
};

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init( argc, argv, "elikos_detection" );

  	string extension = ".calib";
  	string inputFile, outputFile;


    ros::NodeHandle nh_;

	  string calibFolderPath;
    if (!nh_.getParam("/"+ros::this_node::getName()+"/dir", calibFolderPath))
  	{
  		calibFolderPath = "";
  	}
    if (!nh_.getParam("/"+ros::this_node::getName()+"/file_in", inputFile))
  	{
  		inputFile = "calibration_initial";
  	}
    if (!nh_.getParam("/"+ros::this_node::getName()+"/file_out", outputFile))
  	{
  		outputFile = "new_calibration";
  	}
    MessageHandler messageHandler(calibFolderPath+inputFile+extension);

    bool calibrationOn;
    nh_.getParam("/"+ros::this_node::getName()+"/calibration", calibrationOn);

    Tmp tmp;
    remote_calib::Calibrator<elikos_remote_calib_client::CalibPreprocessing> calibrateur(tmp, calibFolderPath);

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
