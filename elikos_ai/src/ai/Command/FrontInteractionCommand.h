//
// Created by olivier on 08/07/16.
//

#ifndef ELIKOS_AI_FRONTINTERACTIONCOMMAND_H
#define ELIKOS_AI_FRONTINTERACTIONCOMMAND_H

#include "AbstractCommand.h"


namespace ai
{

class FrontInteractionCommand : public AbstractCommand
{
public:

    static constexpr double FORWARD_OFFSET{ 0.50 };
    FrontInteractionCommand(QuadRobot* quad, TargetRobot* target);
    virtual ~FrontInteractionCommand();
    virtual bool execute();

private:
    FrontInteractionCommand() = delete;

};

}

#endif //ELIKOS_AI_FRONTINTERACTIONCOMMAND_H
