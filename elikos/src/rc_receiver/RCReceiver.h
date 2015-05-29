//
// Created by tonio on 28/05/15.
//

#ifndef RCRECEIVER_H
#define RCRECEIVER_H

#include <ros/ros.h>
#include <mavros/RCIn.h>
#include "./../defines.cpp"

enum RCChannel { ROLL,
                 PITCH,
                 YAW,
                 THRUST,
                 OFFBOARD_SWITCH,
                 MODE_SWITCH,
                 POSCTL_SWITCH };

class RCReceiver {
public:
    RCReceiver(ros::NodeHandle* nh);
    ~RCReceiver();

    std::vector<unsigned int> getRCChannels() const;

private:
    void RCCallback(const mavros::RCInConstPtr rc);
    ros::NodeHandle* _nh;
    ros::Subscriber _rc_sub;
    std::vector<unsigned int> _rc_channels;
};


#endif RCRECEIVER_H
