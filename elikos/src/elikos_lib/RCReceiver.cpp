//
// Created by tonio on 28/05/15.
//

#include <elikos_lib/RCReceiver.h>

RCReceiver::RCReceiver(ros::NodeHandle nh) : nh_(&nh) {
    rc_sub_ = nh_->subscribe(TOPIC_NAMES[mavros_rc_in], 1, &RCReceiver::RCCallback, this);
    rc_channels_.reserve(NUMBER_OF_CHANNELS);
}

RCReceiver::~RCReceiver() {}

void RCReceiver::RCCallback(const mavros::RCInConstPtr rc) {
    for (size_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
        rc_channels_[i] = rc->channels[i];
    }
}

std::vector<unsigned int> RCReceiver::getRCChannels() const {
    return rc_channels_;
}

unsigned int RCReceiver::operator[](std::size_t i) const {
    return rc_channels_[i];
}