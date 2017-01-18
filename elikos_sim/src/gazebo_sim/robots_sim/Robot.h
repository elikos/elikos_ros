#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/SetModelState.h>

const int ROTATION_ANGLE = 15;
const int ROTATION_SPEED = 5;
const int RANDOM_ANGLE_TIME_LAP = 5;
const std::string REFERENCE_FRAME = "world";

class Robot 
{
public:
    Robot(std::string name, double x, double y, double speed, double updateRate, ros::ServiceClient client);

    virtual void move();

private:
    double _speed;
    int _updateRate;
    double _angle;
    int _timeCounter;
    ros::NodeHandle _nh;
    ros::ServiceClient _client;
    gazebo_msgs::SetModelState _setmodelstate;
    bool _isRotating;
    int _randomBit;
    int _beginRotationTimerCounter;
};
#endif