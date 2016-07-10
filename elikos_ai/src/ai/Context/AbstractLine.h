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

    void evaluate(const TargetRobot& robot);

    virtual ~AbstractLine() = 0;

protected:
    util::Segment segment_;

    virtual bool isGoodLineIntersection( const TargetRobot& robot) = 0;
};

}
#endif // AI_ABSTRACT_LINE_H
