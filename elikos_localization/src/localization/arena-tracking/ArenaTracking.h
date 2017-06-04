#ifndef ARENA_TRACKING_H
#define ARENA_TRACKING_H

#include "DetectedIntersection.h"

namespace tracking
{

class ArenaTracking
{
public:
    ArenaTracking() = default;
    ~ArenaTracking() = default;

    void receiveIntersectionDetection(const std::vector<DetectedIntersection>& intersections);
private:     

};

}

#endif // ARENA_TRACKING_H