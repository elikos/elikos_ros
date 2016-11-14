#include <memory>
#include "MessageHandler.h"
#include "gui/headers/CalibrationWindow.hpp"
#include "gui/headers/ControlWindow.hpp"

int main(int argc, char *argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_remotecalib");
    
    CalibrationWindow calibWindow("Remote Calibration Window");
    ControlWindow controlWindow("Control Window");
    
    calibWindow.setControlWindow(&controlWindow);
    calibWindow.initValeursH();

    MessageHandler messageHandler(calibWindow, controlWindow);

    ros::Rate r(30);

    // Endless loop
    while (ros::ok())
    {
        ros::spinOnce();

        char key = char(cv::waitKey(30));

        calibWindow.keyPressed(key);
        controlWindow.keyPressed(key);

        if (key == 27)
        {
            ros::shutdown();
        }
        else if (key == 's')
        {
            messageHandler.saveCalibration();
        }
        r.sleep();
    }
}