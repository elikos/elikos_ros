//
// Created by olivier on 28/06/16.
//

#ifndef ELIKOS_AI_OBSERVATION_STATE_H
#define ELIKOS_AI_OBSERVATION_STATE_H

#include "AbstractCommand.h"

namespace ai
{

class CommandQueue;

class ObservationCommand : public AbstractCommand
{
public:

    static const tf::Point OBSERVATION_POSITION;

    ObservationCommand(QuadRobot* quad, TargetRobot* target);
    virtual ~ObservationCommand();

    virtual bool isCommmandDone();
    virtual void execute();

private:
    ObservationCommand() = delete;
};

}

#endif //ELIKOS_AI_OBSERVATIONSTATE_H
