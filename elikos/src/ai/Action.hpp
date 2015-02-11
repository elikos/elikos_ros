/**
* @file		ai/Action.hpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#ifndef AI_ACTION_HPP
#define AI_ACTION_HPP

namespace elikos_ai {

/**
 * @class	namespace::Action	Action.hpp	"Definition"
 * @brief	class for an action encapsulation of the AI agent
 */

class Action
{
public:

	// Pose stamped for MAVROS
	geometry_msgs::PoseStamped posStamped;

private:



}; // class elikos_ai::Action

} // namespace elikos_ai

#endif // AI_ACTION_HPP
