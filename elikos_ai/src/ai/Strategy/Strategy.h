#ifndef AI_STRATEGY
#define AI_STRATEGY

#include "RobotTypes.h"

namespace ai {

class Strategy
{
public:
    static const int N_TARGETS = 10;

    Strategy(QuadRobot& quad);
    virtual ~Strategy() = 0;

    // This is the core of the strategy class, it has to be implemented by any concrete strategy.
    virtual Robot* findTargetSelection() = 0;

    inline void updateTarget(const uint8_t& id, const uint8_t& color, const tf::Pose& pose);

protected:
    QuadRobot& quad_;
    // Maybe use
    std::vector<TargetRobot> targets_;

    inline void resetTarget(const int& id);
};

inline void Strategy::resetTarget(const int& id)
{
    targets_[id].updatePositionRadius(30);
    targets_[id].setIsUpdated(false);
}

inline void Strategy::updateTarget(const uint8_t& id, const uint8_t& color, const tf::Pose& pose)
{
    targets_[id].setPose(pose);
    targets_[id].setColor(color);
    targets_[id].setIsUpdated(true);
}

};

#endif /// AI_STRATEGY