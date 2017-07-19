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
    void ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory);
    CmdFrontInteraction::InteractionState getStatus();

    void stateCallBack(const mavros_msgs::State::ConstPtr& msg);

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

    ros::Subscriber stateSub_;
    ros::ServiceClient armingClient_;
    ros::ServiceClient setModeClient_;
    mavros_msgs::SetMode offbSetMode_;
    mavros_msgs::CommandBool armCmd_;

    mavros_msgs::State currentState_;

    double threshold_;
    const double HIGH_OF_GROUND = 0.1;
    double interaction_altitude_;
    double takeoff_altitude_;

    InteractionState interactionStatus_ = InteractionState::LANDING;
    
};

#endif /// CMD_FRONT_INTERACTION_H
