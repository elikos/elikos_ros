//
// Created by olivier on 08/07/16.
//

#ifndef AI_FRONT_INTERACTION_COMMAND_H
#define AI_FRONT_INTERACTION_COMMAND_H

#include "AbstractCommand.h"


namespace ai
{

class FrontInteractionCommand : public AbstractCommand
{
public:
    static const double WAIT_TIME;
    static const double FORWARD_OFFSET;

    FrontInteractionCommand(QuadRobot* quad, TargetRobot* target);
    virtual ~FrontInteractionCommand();

    virtual bool isCommmandDone();
    virtual void execute();


private:
    FrontInteractionCommand() = delete;
    tf::Vector3 destination_;

};

}

#endif // AI_FRONT_INTERACTION_COMMAND_H
