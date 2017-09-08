#ifndef CMD_TOP_INTERACTION_H
#define CMD_TOP_INTERACTION_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdTopInteraction : public CmdAbs
{
public:
    CmdTopInteraction(ros::NodeHandle* nh, int id);
    virtual ~CmdTopInteraction();

    virtual void execute();
    virtual void abort();
    virtual void ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory);

private:
    CmdTopInteraction() = delete;

    tf::StampedTransform targetPosition_;
    tf::StampedTransform lastPosition_;

    const double HIGH_OF_ROBOT = 0.1;

    enum InteractionState {
        LANDING,
        HAS_TOUCHED_ROBOT,
        DONE
    };

    InteractionState interactionStatus_ = InteractionState::LANDING;
    
    double threshold_;
    double takeoff_altitude_;
    double interaction_altitude_;
};

#endif /// CMD_TOP_INTERACTION_H
