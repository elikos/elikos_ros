/**
 * \file BoundedPID.cpp
 * \brief BoundedPID class implementation
 * \author christophebedard
 */

#include "BoundedPID.h"

BoundedPID::BoundedPID(double minCmd, double maxCmd, double p, double i, double d, double i_max, double i_min, bool antiwindup)
    : Pid(p, i, d, i_max, i_min, antiwindup),
      minCmd_(minCmd),
      maxCmd_(maxCmd)
{
}

BoundedPID::~BoundedPID() {}


double BoundedPID::computeCommand(double error, ros::Duration dt) {
    double cmd = Pid::computeCommand(error, dt);
    
    // check if command is greater than min/max
    if (cmd > maxCmd_) {
        cmd = maxCmd_;
    } else if (cmd < minCmd_) {
        cmd = minCmd_;
    }

    return cmd;
}