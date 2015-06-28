//
// Created by tonio on 28/05/15.
//

#ifndef RCRECEIVER_H
#define RCRECEIVER_H

#include <ros/ros.h>
#include <mavros/RCIn.h>
#include "./../../src/defines.cpp"

enum RCChannel { ROLL,
                 PITCH,
                 YAW,
                 THRUST,
                 MODE_SWITCH,
                 OFFBOARD_SWITCH,
                 POSCTL_SWITCH,
                 NUMBER_OF_CHANNELS};

class RCReceiver {
public:
    RCReceiver(ros::NodeHandle nh);
    ~RCReceiver();

    std::vector<unsigned int> getRCChannels() const;
    unsigned int operator[](std::size_t i) const;

private:
    void RCCallback(const mavros::RCInConstPtr rc);
    ros::NodeHandle* nh_;
    ros::Subscriber rc_sub_;
    std::vector<unsigned int> rc_channels_;
};


#endif