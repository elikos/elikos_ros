/*
 * Agent.cpp
 *
 *  Created on: Jan 10, 2015
 *  Author: Myriam Claveau-Mallet
 */


#include <cmath>
#include <boost/bind.hpp>
#include "Agent.h"
#include "./../defines.cpp"


namespace elikos_ai {

/* *************************************************************************************************
 * ***           PUBLIC FUNCTIONS
 * *************************************************************************************************
 */

Agent::Agent( ros::NodeHandle* nh ) : nh_(*nh)
{
    angle_ = 0.0;
    action_ = 0;
}

Agent::~Agent()
{
    //nh_ = 0;
}

void Agent::init()
{
    setPublishers();
    setSubscribers();

    internalModel_ = new InternalModel();
    action_ = new Action();
    ROS_INFO_STREAM("AI INIT");
}

void Agent::destroy()
{
    removePublishers();
    removeSubscribers();

    delete action_;
    delete internalModel_;
    //nh_ = 0;
}


/* *************************************************************************************************
 * ***           DECISION-MAKING - SPECIFIC FUNCTION
 * *************************************************************************************************
 */

bool Agent::hasTarget()
{
    // if there is a robot in sight (any one)
    if ( internalModel_->robots.size() > 1 ) // robots contains at least the quad
    {
        // and if we do not have a target yet
        if ( !hasTarget_ )
        {
            // then this robot is the target

            // TOTEST: target assignation gets done properly
            InternalModel::RobotsMapIterator it = internalModel_->robots.begin();

            if ( it->first == internalModel_->selfId ) // we do not want to aim at ourselves...
            {
                ++it;
            }

            targetRobotId_ = it->first;
            targetRobot_ = it->second;

            hasTarget_ = true;
        }
    }
    else // otherwise
    {
        if ( hasTarget_ )
        {
            // we remove the old target, which is not valid anymore
            targetRobotId_ = -100;
            targetRobot_ = 0;

            hasTarget_ = false;
        }
    }

    return hasTarget_;
}

bool Agent::targetTouched()
{
    // TODO
    // façon facile de faire ça : cheatter avec les mgs ROS (RobotsPos) et indiquer si y'a un overflow sur
    // les côtés de la caméra
    return false;
}

bool Agent::targetCenteredInCamera()
{
    // TODO: les dimensions de la caméra sont pour le moment hardcodées et fausses.
    // J'ai besoin de la largeur et de la hauteur de la caméra...
    double widthCam = CAM_WIDTH;
    double heightCam = CAM_HEIGHT;

    // Et j'ai besoin de ça aussi
    double acceptableCenterWidth = widthCam / 4.0;
    double acceptableCenterHeigth = heightCam / 4.0;

    double acceptableCenterWidthStartPt  = (widthCam  - acceptableCenterWidth)  / 2.0;
    double acceptableCenterHeightStartPt = (heightCam - acceptableCenterHeigth) / 2.0;

    double acceptableCenterWidthEndPt  = acceptableCenterWidthStartPt  + acceptableCenterWidth;
    double acceptableCenterHeightEndPt = acceptableCenterHeightStartPt + acceptableCenterHeigth;

    const tf::Vector3 targetPos = targetRobot_->Transform().getOrigin();

    // Conditions strictement [plus petite, plus grande] ou non strictement? Est-ce que ça change vrmt de quoi?
    bool isInCenterWidth  = ( acceptableCenterWidthStartPt  < targetPos[0] ) && ( targetPos[0] < acceptableCenterWidthEndPt );
    bool isInCenterHeight = ( acceptableCenterHeightStartPt < targetPos[1] ) && ( targetPos[1] < acceptableCenterHeightEndPt );

    return ( isInCenterWidth && isInCenterHeight ) ;
}

void Agent::executePlan()
{
    if ( hasTarget() ) // redundant, but safer
    {
        // (1) first part of plan: move towards the target and "touch" it (in fact, we
        //     will stop right before any actual touch to avoid problems :D
        if ( !targetTouched() )
        {
            // (1.1) if the quad is not directly over the robot, then we move horizontally to be
            //       right above the robot
            if ( !targetCenteredInCamera() )
            {
                // TODO
                // move towards the center of the robot

                // NOTE IMPORTANTE : la position du quad et du robot dans le internalModel_ ne sont pas
                // dans le même système de coordonnées. La position du quad provient de MAVROS tandis
                // que la position des robots est simplement la position du centre du blob DANS L'IMAGE.

                // TODO: update camera width and height
                double widthCam = CAM_WIDTH;
                double heightCam = CAM_HEIGHT;

                tf::Vector3 posRobot = targetRobot_->Transform().getOrigin();
                tf::Vector3 centreImage( widthCam/2.0, heightCam/2.0, 0.0 );
                tf::Vector3 direction = posRobot - centreImage;

                //ROS_INFO_STREAM( "now moving horizontally. direction: " << direction[0] << ", " << direction[2] << ", " << direction[3] );

                // normalize direction
                direction = direction / ( sqrt( pow( (double)direction[0], 2) + pow( (double)direction[1], 2) ) ); // ignore z coor

                double unitMvmt = 0.3; // the length we want to move; we want to move slowly

                //ROS_INFO_STREAM( "now moving horizontally. direction normalized: " << direction[0] << ", " << direction[2] << ", " << direction[3] );

                tf::Vector3 mvmtToDo = direction * unitMvmt;

                //ROS_INFO_STREAM( "now moving horizontally. mvmtToDo: " << mvmtToDo[0] << ", " << mvmtToDo[2] << ", " << mvmtToDo[3] );

                // but we know the camera is fixed on the quad, which has its own coordinate system.
                // we have to change from one coordinate system to the other
                // mainly, we rotate the point around since theorically only the z-axis rotation should change
/*
                double zRotation = 0; // 1 ->> 180 degres
                tf::Quaternion rotation(0, 0, zRotation, 0);
                mvmtToDo = tf::quatRotate(rotation, mvmtToDo);
*/
                action_->posStamped = addRelativeDistToPoseStamped( mvmtToDo[0], mvmtToDo[1], 0.0, internalModel_->self );

                ROS_INFO_STREAM( "now moving horizontally. Movement to do: " << mvmtToDo[0] << ", " << mvmtToDo[1] << ", " << mvmtToDo[2] );
                //ROS_INFO_STREAM( "now moving horizontally. Pos stamped action: " << action_->posStamped.pose.position.x << ", " << action_->posStamped.pose.position.x << ", " << action_->posStamped.pose.position.x );
            }
            // (1.2) when the quad is right above the robot, then it can slowly move down
            else
            {
                // TODO: movement down value
                // move down
                float movmtDown = -1.0;
                action_->posStamped = addRelativeDistToPoseStamped( 0.0, 0.0, movmtDown, internalModel_->self );

                ROS_INFO_STREAM( "now moving down" );
            }
        }
        //  (2) second part of the plan: move back up, and wait there
        else
        {
            ROS_INFO_STREAM( "final step -nothing happens" );
            // TODO
        }
    }
}

void Agent::wanderAround()
{
    // we just wander around randomly
    // TODO
}


/* *************************************************************************************************
 * ***           DECISION-MAKING (THE AI) - MAIN FUNCTIONS
 * *************************************************************************************************
 */

void Agent::percept()
{

    // TODO: protect this call from multi-threads problems ->> the queue could be used in the
    //       ROS subscribers callbacks (this.receiveRobotsPosCallback) and in this function.

    // This call empties the queue of latest received robots positions from the robotDetect node
    // and updates the Agent's internal model.
    internalModel_->updateRobotsPos( queueRobotsPos_ );
    // TOTEST: the whole quad position update thread, from reception in the callback to the update in the internal model
    internalModel_->updateQuadPos( queueQuadPos_ );

}

void Agent::chooseAction()
{
    // For test on Sunday March the 8th, 2015
    if ( hasTarget() )
    {
        executePlan();
    }
    else
    {
        //ROS_INFO_STREAM( "wander around" );
        wanderAround();
    }

    /*
    // Mouvement en forme de "8"
    action_->posStamped = getPoseStamped(angle_);

    // Next step
    angle_ += PI/120;
    if(angle_ >= 2*PI)
    {
        angle_ = 0.0;
    }
    */
}


void Agent::executeAction()
{
    // Publish on MAVROS
    std::string topic = TOPICS_NAMES[mavros_setpoint_local_position];
    rosPublishers_[topic].publish(action_->posStamped);

    //ROS_INFO_STREAM( "Sending new pos for the quad. New pos : " << action_->posStamped.pose.position.x << ", " << action_->posStamped.pose.position.y << ", " << action_->posStamped.pose.position.z );
}


/* *************************************************************************************************
 * ***           ROS SUBSCRIBERS CALLBACKS
 * *************************************************************************************************
 */

/**
 * @fn       void receiveRobotsPosCallback( const elikos_ros::RobotsPos& msg )
 * @brief    Callback for robotsPos topic
 *
 * Transfers RobotsPos messages to a queue when they are received. This queue serves as a container
 * for messages until they are used in the percept() method. Then the queue is emptied.
 *
 */
void Agent::receiveRobotsPosCallback( const elikos_ros::RobotsPos& msg )
{
    //std::cout << "AGENT CALLBACK" << std::endl;
    //ROS_INFO_STREAM( "Agent::callback -- Push RobotsPos message" );
    ROS_INFO_STREAM("CALL BACK");
    queueRobotsPos_.push_back( msg );
}

void Agent::mavrosPoseCallback( const geometry_msgs::PoseStamped::ConstPtr& msg )
{
    // ROS_INFO_STREAM( "AGENT CALLBACK MAVROS position" );
    queueQuadPos_.push_back( msg );
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
    // Orders given to MavROS
    std::string topicName = TOPICS_NAMES[mavros_setpoint_local_position];
    ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(topicName, 1);
    rosPublishers_.insert( std::pair<std::string,ros::Publisher>(topicName, pose_pub) );
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
    ros::Subscriber sub1 = nh_.subscribe( robotsPosTopic, 1000, &Agent::receiveRobotsPosCallback, this );
    rosSubscribers_[robotsPosTopic] = sub1;

    // MAVROS position (la position courante du quad)
    std::string mavrosPosition = TOPICS_NAMES[mavros_setpoint_local_position]; // mavros_setpoint_local_position
    ros::Subscriber sub2 = nh_.subscribe( mavrosPosition, 1000, &Agent::mavrosPoseCallback, this );
    rosSubscribers_[mavrosPosition] = sub2;
}

/**
 * @fn       void removePublishers()
 * @brief    Removes the Agent's publishers to ROS.
 */
void Agent::removePublishers()
{
    // Orders given to MavROS
    // TOTEST:
    for (std::map<std::string,ros::Publisher>::iterator it = rosPublishers_.begin(); it != rosPublishers_.end(); ++it)
    {
        it->second.shutdown();
    }

    rosPublishers_.clear();
}

/**
 * @fn       void removeSubscribers()
 * @brief    Removes the Agent's subscribers to ROS.
 */
void Agent::removeSubscribers()
{
    // Subscribe to all robots' positions' topics
    // TOTEST:
    for (std::map<std::string,ros::Subscriber>::iterator it = rosSubscribers_.begin(); it != rosSubscribers_.end(); ++it)
    {
        it->second.shutdown();
    }

    rosSubscribers_.clear();
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

// NON ce truc est MAL

// adds a relative distance to the quad towards the target robot
geometry_msgs::PoseStamped Agent::addRelativeDistToPoseStamped( float x, float y, float z, Robot* target )
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    if ( target )
    {
        pose.pose.position.x = target->Transform().getOrigin()[0] + x;
        pose.pose.position.y = target->Transform().getOrigin()[1] + y;
        pose.pose.position.z = target->Transform().getOrigin()[2] + z;
    }
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    return pose;
}




} // namespace elikos_ai
