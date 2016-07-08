//
// Created by olivier on 01/07/16.
//

#ifndef AI_GREEN_LINE_H
#define AI_GREEN_LINE_H

#include "AbstractLine.h"

namespace ai
{

class GreenLine : public AbstractLine
{
public:
    GreenLine(const tf::Point &cornerA, const tf::Point &cornerB);
    ~GreenLine() = default;

    virtual void evaluate(const TargetRobot& robot, TargetOrientationEvaluation& evaluation);

private:
    GreenLine() = delete;
};

}

#endif // AI_GREEN_LINE_H
