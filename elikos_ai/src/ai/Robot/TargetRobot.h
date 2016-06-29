#ifndef AI_ROBOT_TARGET_H
#define AI_ROBOT_TARGET_H

#include "Robot.h"

namespace ai
{

class TargetRobot : public Robot
{
public:
    TargetRobot() = default;
    TargetRobot(uint8_t id, uint8_t color);
    virtual ~TargetRobot();

    inline void setId(uint8_t id);
    inline uint8_t getId() const;

    inline void setColor(uint8_t color);
    inline uint8_t getColor() const;

    inline void setPriority(double priority);
    inline double getPriority() const;


private:
    uint8_t id_;
    uint8_t color_;
    double priority_;
};

inline void TargetRobot::setId(uint8_t id)
{
    id_ = id;
}

inline uint8_t TargetRobot::getId() const
{
   return id_;
}

inline double TargetRobot::getPriority() const
{
    return priority_;
}

inline void TargetRobot::setPriority(double priority)
{
    priority_ = priority;
}

inline void TargetRobot::setColor(uint8_t color)
{
    color_ =  color;
}

inline uint8_t TargetRobot::getColor() const
{
    return color_;
}

}

#endif /// AI_ROBOT_TARGET_H
