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

	virtual ~Robot() {}

    void updateRelativePosition( tf::Vector3 position, float orientation );

	const int   getID()             { return this->id_; }
    //      int   getID()             { return this->id_; }
	const float getOrientation()    { return orientation_; }

	const tf::Transform& Transform() { return transform_; }

	void setOrientation( float o )  { orientation_ = o; }

protected:

	//TODO: constructeur avec position relative
    Robot ( int id, tf::Vector3 relativePosition, float orientation, tf::Vector3 speed, RobotType type );

	//TODO: constructeur avec position absolue
    //Robot(tf::Transform absolutePosition);

	//(x,y,z,h) position
	tf::Transform transform_ ;

	//oz orientation
	float orientation_;

	//(vx, vy) vitesse
	tf::Vector3 speed_;

	//type
	RobotType type_;

	// id
	int id_;

private:

    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    Robot();
	Robot& operator= (const Robot&);
	Robot (const Robot&);

}; // class elikos_ai::Robot

} // namespace elikos_ai

#endif // AI_ROBOT_HPP
