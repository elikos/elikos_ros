#include <memory>
#include "MessageHandler.h"
#include "gui/WindowCV.hpp"

int main(int argc, char *argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_remotecalib");

    WindowCV calibWindow;
    MessageHandler messageHandler(calibWindow);

    ros::Rate r(30);

    // Endless loop
    while (ros::ok())
    {
        ros::spinOnce();

        char key = char(cv::waitKey(30));
        calibWindow.keyPressed(key);
        if (key == 27)
        {
            ros::shutdown();
        }else if (key == 's'){
            messageHandler.saveCalibration();
        }
        r.sleep();
    }
}