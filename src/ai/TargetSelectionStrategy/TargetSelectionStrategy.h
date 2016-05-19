#ifndef AI_STRATEGY
#define AI_STRATEGY

#include "RobotTypes.h"

namespace ai {

class TargetSelectionStrategy
{
public:
    static const int N_TARGETS = 10;

    TargetSelectionStrategy(QuadRobot& quad);
    virtual ~TargetSelectionStrategy() = 0;

    // This is the core of the strategy class, it has to be implemented by any concrete strategy.
    virtual Robot* findTargetSelection() = 0;

    inline void updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);

protected:
    QuadRobot& quad_;
    std::vector<TargetRobot> targets_;

    inline void resetTarget(const int& id);
};

inline void TargetSelectionStrategy::resetTarget(const int& id)
{
    targets_[id].updatePositionRadius(30);
    targets_[id].setIsUpdated(false);
}

inline void TargetSelectionStrategy::updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation)
{
    targets_[id].setPosition(position);
    targets_[id].setOrientation(orientation);
    targets_[id].setIsUpdated(true);
}

};
#endif /// AI_STRATEGY
