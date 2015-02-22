/**
* @file		ai/internalModel/Robot.hpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#ifndef AI_ROBOT_HPP
#define AI_ROBOT_HPP

#include "./../../defines.cpp"
#include <elikos_ros/RobotPos.h>
#include <tf/tf.h>

namespace elikos_ai {

/**
 * @class	elikos_ai::Robot	Robot.hpp	"Definition"
 * @brief	Base class for Robot in the ai's internal model
 */


class Robot
{
public:

	virtual ~Robot() {} // virtual destructor


	int getID(){return this->id_;}

protected:

	//TODO: constructeur avec position relative
    Robot ( int id, tf::Point relativePosition, float orientation, tf::Vector3 speed, robotTypes type );

	//TODO: constructeur avec position absolue
    //Robot(tf::Transform absolutePosition);

	//(x,y,z,h) position
	tf::Transform transform_ ;

	//oz orientation
	float orientation_;

	//(vx, vy) vitesse
	tf::Vector3 speed_;

	//type
	robotTypes type_;

	// id
	int id_;

private:

	Robot(); //constructor
}; // class elikos_ai::Robot

} // namespace elikos_ai

#endif // AI_ROBOT_HPP
