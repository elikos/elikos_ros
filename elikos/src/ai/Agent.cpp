/*
 * Agent.cpp
 *
 *  Created on: Jan 10, 2015
 *  Author: Myriam Claveau-Mallet
 */

#include <elikos_ros/RobotsPos.h>

#include "Agent.h"
#include "./../defines.cpp"
//#include "../../../../../devel/include/elikos_ros/RobotsPos.h"

namespace elikos_ai {

/* *************************************************************************************************
 * ***           PUBLIC FUNCTIONS
 * *************************************************************************************************
 */

Agent::Agent( ros::NodeHandle *nh ) : nh_(nh)
{
    angle_ = 0.0;
    action_ = 0;
}

Agent::~Agent()
{
    nh_ = 0;
}

void Agent::init()
{
    setPublishers();
    setSubscribers();

    action_ = new Action();
}

void Agent::destroy()
{
    removePublishers();
    removeSubscribers();

    delete action_;
    nh_ = 0;
}


/* *************************************************************************************************
 * ***           DECISION-MAKING (THE AI)
 * *************************************************************************************************
 */

void Agent::percept()
{
    // TODO: do nothing for now!
}

void Agent::chooseAction()
{
    // Mouvement en forme de "8"
    action_->posStamped = getPoseStamped(angle_);

    // Next step
    angle_ += PI/120;
    if(angle_ >= 2*PI)
    {
        angle_ = 0.0;
    }
}


void Agent::executeAction()
{
    // Publish on MAVROS
    std::string topic = TOPICS_NAMES[mavros_setpoint_local_position];
    rosPublishers_[topic].publish(action_->posStamped);
}

/* *************************************************************************************************
 * ***           ROS RELATED FUNCTIONS
 * *************************************************************************************************
 */

/**
 * @fn       void setPublishers()
 * @brief    Tells ROS which topics the Agent wants to publish into.
 *
 * The main topic to publish to: orders given to MAVROS.
 *
 */
void Agent::setPublishers()
{
    if ( nh_ )
    {
        // Orders given to MavROS
        std::string topicName = TOPICS_NAMES[mavros_setpoint_local_position];
        ros::Publisher pose_pub = nh_->advertise<geometry_msgs::PoseStamped>(topicName, 1);
        rosPublishers_.insert( std::pair< std::string, ros::Publisher >( topicName, pose_pub ) );
    }
}

/**
 * @fn       void setSubscribers()
 * @brief    Sets the Agent's subscribers to ROS.
 *
 * The main ROS topics subscribing to: all robots positions topics, MAVROS
 * positions (the quad's).
 *
 */
void Agent::setSubscribers()
{
    // Subscribe to all robots' positions' topics
    std::string robotsPosTopic = TOPICS_NAMES[robotsPos];
    ros::Subscriber sub = nh_->subscribe(robotsPosTopic, 1000, receiveRobotsPos);
    rosSubscribers_[robotsPosTopic] = sub;
}

/**
 * @fn       void removePublishers()
 * @brief    Removes the Agent's publishers to ROS.
 */
void Agent::removePublishers()
{
    // Orders given to MavROS
    // TODO:
}

/**
 * @fn       void removeSubscribers()
 * @brief    Removes the Agent's subscribers to ROS.
 */
void Agent::removeSubscribers()
{
    // Subscribe to all robots' positions' topics
    // TODO:
}

/**
 * @fn       void receiveRobotsPos( const elikos_ros::RobotsPos& msg )
 * @brief    Removes the Agent's subscribers to ROS.
 */
void receiveRobotsPos( const elikos_ros::RobotsPos& msg )
{

}


/* *************************************************************************************************
 * ***           TOOLS
 * *************************************************************************************************
 */

geometry_msgs::PoseStamped Agent::getPoseStamped(float angle)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = cos(angle);
    pose.pose.position.y = sin(2*angle)/2;
    pose.pose.position.z = 1.5 + sin(3*angle)/2;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    return pose;
}



} // namespace elikos_ai
