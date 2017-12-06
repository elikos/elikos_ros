//
// Created by olivier on 01/07/16.
//

#ifndef AI_WHITE_LINE_H
#define AI_WHITE_LINE_H

#include "AbstractLine.h"

namespace ai
{

class WhiteLine : public AbstractLine
{
public:
    WhiteLine(tf::Point& cornerA, tf::Point& cornerB);
    virtual ~WhiteLine();

    virtual bool isGoodLineIntersection(const TargetRobot& robot);

private:
    //WhiteLine() = delete;
};

}

#endif // AI_WHITELINE_H
