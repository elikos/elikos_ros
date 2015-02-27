/**
* @file		ai/internalModel/ObstacleRobot.hpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#ifndef AI_OBSTACLE_ROBOT_HPP
#define AI_OBSTACLE_ROBOT_HPP

#include "Robot.hpp"

namespace elikos_ai {

/**
 * @class	elikos_ai::ObstacleRobot	ObstacleRobot.hpp	"Definition"
 * @brief	class for ObstacleRobot in the ai's internal model
 */

class ObstacleRobot : public Robot
{
public:

    ObstacleRobot( int id, tf::Point relativePosition, float orientation );
    ObstacleRobot( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed ); //constructor
    virtual ~ObstacleRobot() {}


private:

    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    ObstacleRobot();
    ObstacleRobot& operator= (const ObstacleRobot&);
    ObstacleRobot (const ObstacleRobot&);

}; // class elikos_ai::ObstacleRobot

} // namespace elikos_ai

#endif // AI_OBSTACLE_ROBOT_HPP
