//
// Created by olivier on 28/06/16.
//

#ifndef AI_INTERACTION_STATE_H
#define AI_INTERACTION_STATE_H

#include "AbstractCommand.h"

namespace ai
{

class CommandQueue;

class TopInteractionCommand : public AbstractCommand
{
public:
    static constexpr double HEIGHT_OFFSET{ 0.0 };
    static constexpr double WAIT_TIME{ 1.0 };

    TopInteractionCommand(QuadRobot* quad, TargetRobot* target);
    virtual ~TopInteractionCommand();

    virtual bool isCommmandDone();
    virtual void execute();

private:
    TopInteractionCommand() = delete;
};

}

#endif //ELIKOS_AI_INTERACTIONSTATE_H

