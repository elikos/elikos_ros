//
// Created by olivier on 01/07/16.
//

#ifndef AI_ABSTRACT_LINE_H
#define AI_ABSTRACT_LINE_H

#include <tf/tf.h>
#include "Segment.h"
#include "TargetOrientationEvaluation.h"


namespace ai
{

class TargetRobot;

class AbstractLine
{
public:
    AbstractLine();
    AbstractLine(const tf::Point& cornerA, const tf::Point& cornerB);
    bool isInThePath(const TargetRobot& robot) const;
    virtual void evaluate(const TargetRobot& robot, TargetOrientationEvaluation& evaluation) = 0;

    virtual ~AbstractLine() = 0;

protected:
    util::Segment segment_;

};

}
#endif // AI_ABSTRACT_LINE_H
