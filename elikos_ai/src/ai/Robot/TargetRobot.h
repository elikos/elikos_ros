#ifndef AI_ROBOTTARGET_H
#define AI_ROBOTTARGET_H

#include "Robot.h"

namespace ai
{

class TargetRobot : public Robot
{
public:
    TargetRobot() = default;
    TargetRobot(uint8_t id, uint8_t color);
    virtual ~TargetRobot();

    inline uint8_t getId() const;
    inline uint8_t getColor() const;

    inline void setId(uint8_t id);
    inline void setColor(uint8_t color);

private:
    uint8_t id_;
    uint8_t color_;
};

inline void TargetRobot::setId(uint8_t id)
{
    id_ = id;
}

inline uint8_t TargetRobot::getId() const
{
   return id_;
}

inline void TargetRobot::setColor(uint8_t color)
{
    color_ =  color;
}

inline uint8_t TargetRobot::getColor() const
{
    return color_;
}

};

#endif /// AI_ROBOTTARGET_H
