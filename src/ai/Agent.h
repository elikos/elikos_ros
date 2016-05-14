#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include "StateMachine.h"
#include <tf/tf.h>

namespace ai
{

class Agent
{
public:
    void updateTarget(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    void updateObstacle(const int& id, const tf::Vector3& position, const tf::Quaternion& orientation);
    void updateMAV(const tf::Vector3& position, const tf::Quaternion& orientation);

    Agent() = default;
    ~Agent() = default;

private:
    StateMachine stateMachine_;
    

};

}
#endif /// AI_FACADE_H
