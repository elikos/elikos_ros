/**
* @file		ai/internalModel/TargetRobot.hpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#ifndef AI_TARGET_ROBOT_HPP
#define AI_TARGET_ROBOT_HPP

#include "Robot.hpp"

namespace elikos_ai {

/**
 * @class	namespace::TargetRobot	TargetRobot.hpp	"Definition"
 * @brief	class for TargetRobot in the ai's internal model
 */

class TargetRobot : public Robot
{
public:

	TargetRobot(int id, tf::Transform transform, float orientation, tf::Vector3 spped); //constructor

	Color getColor () { return this->color; }

	enum Color {RED, GREEN};

private:

	Color color;


}; // class elikos_ai::TargetRobot

} // namespace elikos_ai

#endif // AI_TARGET_ROBOT_HPP
