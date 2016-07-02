//
// Created by olivier on 01/07/16.
//

#include "GreenLine.h"
#include "WhiteLine.h"
#include "ArenaA.h"

namespace ai
{

ArenaA::ArenaA()
{
    lines_.push_back(std::unique_ptr<GreenLine>(new GreenLine(TOP_LEFT_CORNER, TOP_RIGHT_CORNER)));
    lines_.push_back(std::unique_ptr<WhiteLine>(new WhiteLine(BOTTOM_LEFT_CORNER, TOP_LEFT_CORNER)));
    lines_.push_back(std::unique_ptr<WhiteLine>(new WhiteLine(BOTTOM_RIGHT_CORNER, BOTTOM_LEFT_CORNER)));
    lines_.push_back(std::unique_ptr<WhiteLine>(new WhiteLine(TOP_RIGHT_CORNER, BOTTOM_RIGHT_CORNER)));
}

ArenaA::~ArenaA()
{
}

void ArenaA::evaluateTargetOrientation(const TargetRobot& robot, TargetOrientationEvaluation &evaluation)
{
    for (int i = 0; i < lines_.size(); i++)
    {
        if (lines_[i]->isInThePath(robot))
        {
            lines_[i]->evaluate(robot, evaluation);
            return;
        }
    }
}


}
