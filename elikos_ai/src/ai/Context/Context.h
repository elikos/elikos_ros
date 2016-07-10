//
// Created by olivier on 10/07/16.
//

#ifndef AI_CONTEXT_H
#define AI_CONTEXT_H

#include <memory>

#include "RobotTypes.h"
#include "AbstractArena.h"
#include <elikos_ros/TargetRobotArray.h>

namespace ai
{

class Context
{

public:
    Context();
    ~Context() = default;

    inline QuadRobot& getQuad();
    inline std::vector<TargetRobot>& getTargets();
    inline AbstractArena* getArena();

    TargetRobot* findHighestPriorityTarget();
    void resetPriority();

    void updateTargets(const elikos_ros::TargetRobotArray::ConstPtr& input);

private:
    QuadRobot quad_;
    std::vector<TargetRobot> targets_;
    std::unique_ptr<AbstractArena> arena_;

    void updateTarget(const elikos_ros::TargetRobot& target, int i);
};

inline QuadRobot& Context::getQuad()
{
    return quad_;
}

inline std::vector<TargetRobot>& Context::getTargets()
{
    return targets_;
}

inline AbstractArena* Context::getArena()
{
    return arena_.get();
}

}

#endif // AI_CONTEXT_H
