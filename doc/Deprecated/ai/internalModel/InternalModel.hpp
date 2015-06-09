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
#include <vector>
#include <elikos_ros/RobotsPos.h>
#include <elikos_ros/RobotPos.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

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

    void updateRobotsPos( std::vector<elikos_ros::RobotsPos>& robotsMsgs );
    void updateQuadPose( std::vector<geometry_msgs::PoseStamped::ConstPtr>& poseMsgs,
                         std::vector<sensor_msgs::Imu::ConstPtr>& imuMsgs);


    /* *************************************************************************************************
     * ***           PUBLIC ATTRIBUTES
     * *************************************************************************************************
     */

    typedef std::map<int, Robot*>           RobotsMap;
    typedef std::map<int, Robot*>::iterator RobotsMapIterator;

    RobotsMap robots;             /**< map for keeping track of robots */
                                  /**< @note map<unique id, Robot*> */
    Robot* self;                  /**< the quad itself */
                                  /**< @note this Robot* is in the std::map robots */
                                  /**< @note ID number reserved: -1 */
    int selfId = -1;              /**< ID reserved for the quad robot. */

private:

    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    InternalModel& operator= (const InternalModel&);
    InternalModel (const InternalModel&);

}; // class elikos_ai::InternalModel

} // namespace elikos_ai

#endif // AI_INTERNAL_MODEL_HPP
