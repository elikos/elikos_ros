#ifndef AI_STRATEGY
#define AI_STRATEGY

#include "TargetRobot.h"

namespace ai {

class TargetSelectionStrategy
{
public:
    static const int N_TARGETS = 10;

    TargetSelectionStrategy();
    virtual ~TargetSelectionStrategy() = 0;

    // This is the core of the strategy class, it has to be implemented by any concrete strategy.
    virtual void updateTargetSelection() = 0;

    inline void updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    inline Robot* getTargetSelection();

private:
    std::vector<TargetRobot> targets_;
    Robot* selectedTarget_;
};

inline void TargetSelectionStrategy::updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation)
{
    targets_[id].setPosition(position);
    targets_[id].setOrientation(orientation);
}

inline Robot* TargetSelectionStrategy::getTargetSelection()
{
    return selectedTarget_;
}

};
#endif /// AI_STRATEGY
