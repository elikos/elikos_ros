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

class AbstractLine : public util::Segment
{
public:
    AbstractLine();
    AbstractLine(tf::Point& cornerA, tf::Point& cornerB);
    bool isInThePath(const TargetRobot& robot) const;

    virtual bool isGoodLineIntersection(const TargetRobot& robot) = 0;
    void evaluate(TargetRobot& robot);

    virtual ~AbstractLine() = 0;
};

}
#endif // AI_ABSTRACT_LINE_H
