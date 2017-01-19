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

    MessageHandler messageHandler(calibWindow, controlWindow);

    ros::Rate refreshRate(30);

    // Endless loop
    while (ros::ok())
    {
        ros::spinOnce();

        char key = char(cv::waitKey(30));

        calibWindow.keyPressed(key);
        controlWindow.keyPressed(key);

        if (key == 27)//Escape
        {
            ros::shutdown();
        }
        else if (key == 's')
        {
            messageHandler.saveCalibration();
        }else if (key == 'r'){
            messageHandler.sendRefreshCommand();
        }
        refreshRate.sleep();
    }
}