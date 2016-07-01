//
// Created by olivier on 27/06/16.
//

#ifndef AI_ABSTRACT_CONSIDERATION_H
#define AI_ABSTRACT_CONSIDERATION_H

#include "RobotTypes.h"
#include <vector>

namespace ai
{


class AbstractConsideration
{
public:

    static constexpr double MAX_DISTANCE { 20.0 };
    static constexpr double LEFT_SIDE  { -10.0 };
    static constexpr double RIGHT_SIDE {  10.0 };
    static constexpr double UP_SIDE    {  10.0 };
    static constexpr double DOWN_SIDE  { -10.0 };

    static const int N_CORNERS { 4 };
    static const tf::Vector3 corners[N_CORNERS];

    AbstractConsideration() = default;
    virtual ~AbstractConsideration() = 0;
    virtual void evaluatePriority(std::vector<TargetRobot>& targets, const QuadRobot& quad) = 0;
};

}

#endif //AI_ABSTRACT_CONSIDERATION_H
