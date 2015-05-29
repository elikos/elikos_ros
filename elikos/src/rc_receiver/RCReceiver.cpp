//
// Created by tonio on 28/05/15.
//

#include "RCReceiver.h"

RCReceiver::RCReceiver(ros::NodeHandle* nh) : _nh(nh) {
    _rc_sub = _nh->subscribe(TOPICS_NAMES[mavros_rc_in], 1, this->RCCallback);
}

RCReceiver::~RCReceiver() {}

void RCReceiver::RCCallback(const mavros::RCInConstPtr rc) {
    _rc_channels.clear();
    for (auto channel : rc->channels) {
        _rc_channels.push_back(channel);
    }
}

std::vector<unsigned int> RCReceiver::getRCChannels() const {
    return _rc_channels;
}