/**
* @file		ai/internalModel/InternalModel.hpp
* @author	Myriam Claveau-Mallet
* @date		02/15/2015
*/

#ifndef AI_INTERNAL_MODEL_HPP
#define AI_INTERNAL_MODEL_HPP

#include "Robot.hpp"
#include "TargetRobot.hpp"
#include "ObstacleRobot.hpp"

#include <map>
#include <queue>
#include <vector>
#include <elikos_ros/RobotsPos.h>
#include <elikos_ros/RobotPos.h>

namespace elikos_ai {

/**
 * @class	elikos_ai::InternalModel	InternalModel.hpp	"Definition"
 * @brief	container class for the ai's internal model
 */

class InternalModel
{
public:

    InternalModel();
    ~InternalModel();

    /* *************************************************************************************************
     * ***           PUBLIC FUNCTIONS
     * *************************************************************************************************
     */

    void updateModel( std::queue<elikos_ros::RobotsPos> robotsMsgs );


    /* *************************************************************************************************
     * ***           PUBLIC ATTRIBUTES
     * *************************************************************************************************
     */

    std::map<int, Robot*> robots; /**< map for keeping track of robots */
                                  /**< @note map<unique id, Robot*> */

private:

    /* *************************************************************************************************
     * ***           PRIVATE CONSTRUCTORS
     * *************************************************************************************************
     */

    InternalModel( InternalModel& );

}; // class elikos_ai::InternalModel

} // namespace elikos_ai

#endif // AI_INTERNAL_MODEL_HPP
