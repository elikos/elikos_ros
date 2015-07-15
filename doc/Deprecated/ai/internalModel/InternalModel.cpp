/**
* @file		ai/internalModel/InternalModel.cpp
* @author	Myriam Claveau-Mallet
* @date		02/15/2015
*/

#include "InternalModel.hpp"
#include "Factory/RobotsFactory.hpp"
#include <tf/tf.h>


namespace elikos_ai {


InternalModel::InternalModel()
{
    robots[selfId] = RobotsFactory::Instance().newRobot( quadRobot, selfId, tf::Vector3( 0.0, 0.0, 0.0 ), 0.0 );
    self = robots[selfId];
}

InternalModel::~InternalModel()
{
    for ( std::map<int, Robot*>::iterator it = robots.begin(); it != robots.end(); ++it )
    {
        delete it->second;
    }
}

/* *************************************************************************************************
 * ***           PUBLIC FUNCTIONS
 * *************************************************************************************************
 */

/**
* @fn      updateModel( std::queue<elikos_ros::RobotsPos>& robotsMsgs )
* @brief   Updates the internal model with incoming robots positions.
* @note    Empties the queue while updating the internal model.
* @note    The queue parameter HAS to be a copy (it's a multi-threaded environment)
* @param   robotsMsgs  Copy of the queue containing the latest received robots positions messages.
*/
void InternalModel::updateRobotsPos( std::vector<elikos_ros::RobotsPos>& robotsMsgs )
{
    // TODO: must empty the queue and update internal model
    // TOTEST: !! this whole function... making sure with empty the messages' queue correctly

    // Update internal model
    for ( int i = 0; i < robotsMsgs.size(); ++i )
    {
        //elikos_ros::RobotsPos msg = robotsMsgs.front();
        std::vector<elikos_ros::RobotPos> robotsPos = robotsMsgs[i].robotsPos;

        for ( int j = 0; j < robotsPos.size(); ++j )
        {
            elikos_ros::RobotPos& robotPos = robotsPos[j];

            // Check if the robot has already been identified and created
            std::map<int, Robot*>::iterator it = robots.find((int)robotPos.id);

            // TODO: finish this (updating the robots positions, including checking robot type)
            if ( it == robots.end() ) // the robot does not exist yet
            {
                // QUESTION: on ferait pas mieux de juste donner un type aux robots et de laisser faire l'héritage "Robot", "TargetRobot, "ObstacleRobot"?
                // TOTEST: création des robots dans le modèle interne à partir des messages RobotsPos

                robots[(int)robotPos.id] = RobotsFactory::Instance().newRobot( (RobotType)robotPos.type, (int)robotPos.id, tf::Vector3( robotPos.point.x, robotPos.point.y, robotPos.point.z ), (float)robotPos.orientation );

                // DEBUG:
                ROS_INFO_STREAM( "Robot created successfully in the AI's InternalModel. Robot id : " << (int)robotPos.id );
            }
            else
            {
                robots[(int)robotPos.id]->updateRelativePosition( tf::Vector3( robotPos.point.x, robotPos.point.y, robotPos.point.z ), (float)robotPos.orientation );
                //ROS_INFO_STREAM( "Robot's position updated : (" << robotPos.point.x << " ," << robotPos.point.y << ", " << robotPos.point.z << ")" );
            }
        }

        // Clear queue
        //robotsMsgs.pop();
    }

    //ROS_INFO_STREAM( "Received vector size : " << robotsMsgs.size() );

    // CHECKTHIS: This was made without the Internet... No way I could check std::vector in more details.
    robotsMsgs.clear();
}

void InternalModel::updateQuadPose( std::vector<geometry_msgs::PoseStamped::ConstPtr>& poseMsgs,
                                    std::vector<sensor_msgs::Imu::ConstPtr>& imuMsgs)
{
    for (auto msg : poseMsgs) {
        const geometry_msgs::Point &pt = msg->pose.position;
        self->updateRelativePosition( tf::Vector3( pt.x, pt.y, pt.z ), self->getOrientation() );
    }

    for (auto msg : imuMsgs) {
        const geometry_msgs::Quaternion &quat = msg->orientation;
        tf::Quaternion tfQuat;
        tf::quaternionMsgToTF(quat, tfQuat);
        self->updateRelativePosition( self->Transform().getOrigin(), tf::getYaw(tfQuat) );
    }

    //ROS_INFO_STREAM( "Refreasing the quad's position. New pos : " << pt.x << ", " << pt.y << ", " << pt.z );

    poseMsgs.clear();
    imuMsgs.clear();
}


} // namespace elikos_ai