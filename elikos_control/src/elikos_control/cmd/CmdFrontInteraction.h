#ifndef CMD_FRONT_INTERACTION_H
#define CMD_FRONT_INTERACTION_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdFrontInteraction : public CmdAbs
{
public:
    CmdFrontInteraction(ros::NodeHandle* nh, int id);
    virtual ~CmdFrontInteraction();

    virtual void execute();
    virtual void abort();
    virtual void ajustement();

private:
    CmdFrontInteraction() = delete;

    tf::StampedTransform targetPosition_;
    tf::StampedTransform lastPosition_;

    const double THRESHOLD = 0.05;  //TODO fichier de config
    const double HIGH_OF_GROUND = 0.1;

    enum InteractionState {
        LANDING,
        HAS_TOUCHED_GROUND,
        DONE
    };

    InteractionState interactionStatus_ = InteractionState::LANDING;
    
};

#endif /// CMD_FRONT_INTERACTION_H
