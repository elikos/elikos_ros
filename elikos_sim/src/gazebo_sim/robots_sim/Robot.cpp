#include "Robot.h"

Robot::Robot(std::string name, double x, double y, double speed, double updateRate, ros::ServiceClient client): _speed(speed), _updateRate(updateRate), _client(client)
{
    _nh.getParam("/"+ros::this_node::getName()+"/random_angle_time_lap", _random_angle_time_lap);

    _nh.getParam("/"+ros::this_node::getName()+"/rotation_angle", _rotation_angle);

    _nh.getParam("/"+ros::this_node::getName()+"/rotation_speed", _rotation_speed);

    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    _angle = PI;
    if(!((sqrt(pow(x,2)+pow(y,2)) + x)==0))
        _angle = 2*atan(y/(sqrt(pow(x,2)+pow(y,2)) + x));
    
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0, 0, _angle);

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = name;
    modelstate.reference_frame = REFERENCE_FRAME;
    modelstate.pose = pose;
    modelstate.twist = twist;

    _setmodelstate.request.model_state = modelstate;
    _client.call(_setmodelstate);

    _isRotating = false;
}

void Robot::move()
{
    if((_timeCounter % (_updateRate*_random_angle_time_lap))==0 || _isRotating)
    {
        if(!_isRotating)
        {
            _randomBit = rand()%2;
            _beginRotationTimerCounter = _timeCounter;
            _isRotating = true;
        }
        else if((_timeCounter-_beginRotationTimerCounter)/_updateRate >= _rotation_angle/_rotation_speed)
        {
            _isRotating = false;
        }

        if(_randomBit==1)
        {
            _angle += _rotation_speed/_updateRate;
        }
        else
        {
            _angle -= _rotation_speed/_updateRate;
        }
    }
    _timeCounter++;
    
    if(!_isRotating)
    {
        _setmodelstate.request.model_state.pose.position.x += (_speed/_updateRate)*cos(_angle);
        _setmodelstate.request.model_state.pose.position.y += (_speed/_updateRate)*sin(_angle);
        _setmodelstate.request.model_state.twist.linear.x = _speed * cos(_angle);
        _setmodelstate.request.model_state.twist.linear.y = _speed * sin(_angle);
        _setmodelstate.request.model_state.twist.angular.z = 0;
    } else {
        _setmodelstate.request.model_state.twist.linear.x = 0;
        _setmodelstate.request.model_state.twist.linear.y = 0;
        _setmodelstate.request.model_state.twist.angular.z = _randomBit ? _rotation_speed : -_rotation_speed;
    }

    _setmodelstate.request.model_state.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0, 0, _angle);

    _client.call(_setmodelstate);
}