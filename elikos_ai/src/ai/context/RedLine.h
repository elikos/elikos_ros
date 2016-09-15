//
// Created by olivier on 01/07/16.
//

#ifndef AI_RED_LINE_H
#define AI_RED_LINE_H

#include "AbstractLine.h"

namespace ai
{

class RedLine : public AbstractLine
{
public:
    RedLine(const tf::Point& cornerA, const tf::Point& cornerB);
    virtual ~RedLine();

    virtual bool isGoodLineIntersection(const TargetRobot& robot);

private:
    RedLine() = delete;
};

}

#endif //AI_RED_LINE_H
