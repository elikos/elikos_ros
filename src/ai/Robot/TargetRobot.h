#ifndef AI_ROBOTTARGET_H
#define AI_ROBOTTARGET_H

#include "Robot.h"

namespace ai
{

class TargetRobot : public Robot
{
public:
    TargetRobot() = default;
    TargetRobot(const int& id);
    virtual ~TargetRobot() ;
};

}

#endif /// AI_ROBOTTARGET_H
