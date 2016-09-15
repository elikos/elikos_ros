//
// Created by olivier on 01/07/16.
//

#ifndef AI_ABSTRACT_LINE_H
#define AI_ABSTRACT_LINE_H

#include <tf/tf.h>
#include "Segment.h"

namespace ai
{

class TargetRobot;

class AbstractLine
{
public:
    AbstractLine();
    AbstractLine(const tf::Point& cornerA, const tf::Point& cornerB);
    bool isInThePath(const TargetRobot& robot) const;

    virtual bool isGoodLineIntersection(const TargetRobot& robot) = 0;
    void evaluate(TargetRobot& robot);

    virtual ~AbstractLine() = 0;

protected:
    util::Segment segment_;
};

}
#endif // AI_ABSTRACT_LINE_H
