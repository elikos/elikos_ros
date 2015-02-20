/**
* @file		ai/internalModel/Robot.hpp
* @author	Myriam Claveau-Mallet
* @date		02/08/2015
*/

#ifndef AI_ROBOT_HPP
#define AI_ROBOT_HPP

#include "./../defines.cpp"
#include "RobotPos.msg"
namespace elikos_ai {

/**
 * @class	elikos_ai::Robot	Robot.hpp	"Definition"
 * @brief	Base class for Robot in the ai's internal model
 */


class Robot
{
public:

    Robot();
	virtual ~Robot() {} // virtual destructor


	int getID(){return this->id;}

protected:

	//TODO: constructeur avec position relative
    Robot (tf::transform relativePosition);

	//TODO: constructeur avec position absolue
    Robot(tf::Transform absolutePosition);

	//(x,y,z,h) position
	tf::Transform transform_ ;

	//oz orientation
	float orientation_;

	//(vx, vy) vitesse
	tf::Vector3 speed_;

	//type
	RobotTypes type;

	// id
	int id_;

private:

	Robot(); //constructor
}; // class elikos_ai::Robot

} // namespace elikos_ai

#endif // AI_ROBOT_HPP
