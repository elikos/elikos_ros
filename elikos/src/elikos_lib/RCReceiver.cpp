//
// Created by tonio on 28/05/15.
//

#include <elikos_lib/RCReceiver.h>

RCReceiver::RCReceiver(ros::NodeHandle nh) : _nh(&nh) {
    _rc_sub = _nh->subscribe(TOPIC_NAMES[mavros_rc_in], 1, &RCReceiver::RCCallback, this);
    _rc_channels.reserve(NUMBER_OF_CHANNELS);
}

RCReceiver::~RCReceiver() {}

void RCReceiver::RCCallback(const mavros::RCInConstPtr rc) {
    for (size_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
        _rc_channels[i] = rc->channels[i];
    }
}

std::vector<unsigned int> RCReceiver::getRCChannels() const {
    return _rc_channels;
}

unsigned int RCReceiver::operator[](std::size_t i) const {
    return _rc_channels[i];
}