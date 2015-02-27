/**
* @file		ai/internalModel/InternalModel.cpp
* @author	Myriam Claveau-Mallet
* @date		02/15/2015
*/

#include "InternalModel.hpp"


namespace elikos_ai {


/* *************************************************************************************************
 * ***           PUBLIC FUNCTIONS
 * *************************************************************************************************
 */

InternalModel::InternalModel()
{

}

InternalModel::~InternalModel()
{
    for ( std::map<int, Robot*>::iterator it = robots.begin(); it != robots.end(); ++it )
    {
        delete it->second;
    }
}

/**
 * @fn      updateModel( std::queue<elikos_ros::RobotsPos>& robotsMsgs )
 * @brief   Updates the internal model with incoming robots positions.
 * @note    Empties the queue while updating the internal model.
 * @note    The queue parameter HAS to be a copy (it's a multi-threaded environment)
 * @param   robotsMsgs  Copy of the queue containing the latest received robots positions messages.
 */
void InternalModel::updateModel( std::queue<elikos_ros::RobotsPos> robotsMsgs )
{
    // TODO: must empty the queue and update internal model

    // Update internal model
    for ( int i = 0; i < robotsMsgs.size(); ++i )
    {
        elikos_ros::RobotsPos msg = robotsMsgs.front();
        std::vector<elikos_ros::RobotPos> robotsPos = msg.robotsPos;

        for ( int j = 0; j < robotsPos.size(); ++j )
        {
            elikos_ros::RobotPos& robotPos = robotsPos[j];

            // Check if the robot has already been identified and created
            std::map<int, Robot*>::iterator it = robots.find((int)robotPos.id);

            // TODO: finish this (updating the robots positions, including checking robot type)
            if ( it == robots.end() ) // the robot does not exist yet
            {
                robots[(int)robotPos.id] = new TargetRobot();
            }
            else
            {

            }
        }

        // Clear queue
        robotsMsgs.pop();
    }
}


} // namespace elikos_ai
