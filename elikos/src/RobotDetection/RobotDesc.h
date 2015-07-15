#ifndef DETECT_ROBOTDESC_H
#define DETECT_ROBOTDESC_H

#include <opencv2/core/core.hpp>
#include "string"
#include <elikos_ros/RobotPos.h>
#include <elikos_ros/RobotsPos.h>
#include <ros/ros.h>

using namespace std;


class RobotDesc {
public:
    RobotDesc();

    ~RobotDesc();

    int getHPos();

    int getVPos();

    void setXPos(int pos);

    void setYPos(int pos);

    int getXPos();

    int getYPos();

    cv::Scalar getHSVmin();

    cv::Scalar getHSVmax();

    void setHSVmin(cv::Scalar scalar);

    void setHSVmax(cv::Scalar scalar);

    elikos_ros::RobotPos toMsg();

private:
    int xPos, yPos;
    string type;
    cv::Scalar HSVmin, HSVmax;


    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    // TODO: check this out! Copy constructor and operator= get called for class RobotDesc. It's impossible to make these constructor private and not implement them.
    //RobotDesc& operator= (const RobotDesc&);
    //RobotDesc (const RobotDesc&)

};
#endif


