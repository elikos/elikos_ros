#ifndef CMD_FRONT_INTERACTION_H
#define CMD_FRONT_INTERACTION_H

#include <mutex>
#include <condition_variable>
#include <chrono>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdFrontInteraction : public CmdAbs
{
public:

    enum InteractionState {
        LANDING,
        HAS_TOUCHED_GROUND,
        DONE,
        ASKS_FOR_OFFBOARD
    };

    CmdFrontInteraction(ros::NodeHandle* nh, int id);
    virtual ~CmdFrontInteraction();

    virtual void execute();
    virtual void abort();
    virtual void ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory);
    CmdFrontInteraction::InteractionState getStatus();

private:
    CmdFrontInteraction() = delete;

    void TakeABreak();
    std::mutex sleepLock_;
    bool isSleeping_;
    std::condition_variable sleepCV_;
    const std::chrono::duration<int, std::ratio<1, 1>> SLEEP_TIME;
    ros::ServiceClient landingClient_;
    mavros_msgs::CommandTOL landingCmd_;

    tf::StampedTransform targetPosition_;
    tf::StampedTransform lastPosition_;

    const double THRESHOLD = 0.05;  //TODO fichier de config
    const double HIGH_OF_GROUND = 0.1;

    InteractionState interactionStatus_ = InteractionState::LANDING;
    
};

#endif /// CMD_FRONT_INTERACTION_H
