#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <tf/tf.h>
#include <memory>
#include "Timer.h"

namespace ai
{

class Agent;

class CommandQueue;
class TargetRobot;
class QuadRobot;

class AbstractCommand
{
public:
    AbstractCommand(QuadRobot* quad, TargetRobot* target_);
    virtual ~AbstractCommand() = 0;

    virtual void execute() = 0;
    virtual bool isCommmandDone() = 0;

protected:
    QuadRobot* quad_;
    TargetRobot* target_;
    util::Timer timer_;

    bool hasReachedDestination(const tf::Vector3& currentPosition, const tf::Vector3& destination);
    AbstractCommand() = default;
};

}

#endif /// ABSTRACT_STATE_H
