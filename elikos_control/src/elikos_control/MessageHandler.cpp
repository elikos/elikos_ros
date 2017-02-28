#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "MessageHandler.h"



MessageHandler::MessageHandler()
{
	sub_ = nh_.subscribe("elikos_trajectory", 1, &MessageHandler::dispatchMessage, this);
}


MessageHandler::~MessageHandler()
{

}

void MessageHandler::dispatchMessage(const elikos_ros::TrajectoryCmd::ConstPtr& input)
{
	lastReceivedCmd_.cmdCode_ = input->cmdCode;
	lastReceivedCmd_.cmdTrajectory_ = input->trajectory;

	//Ce qui suit est du code temporaire qui assure un contrôle par position.
    /*tf::StampedTransform currentPosition;
	tf_listener_.lookupTransform("elikos_arena_origin", "elikos_base_link",
							ros::Time(0), currentPosition);
	int i = 0;
	while(i < trajectory.points.size()-1)
	{
		geometry_msgs::Vector3 targetTranslation = trajectory.points[i].transforms[0].translation;

		if(pow(targetTranslation.x-currentPosition.getOrigin().x(), 2)+
			pow(targetTranslation.y-currentPosition.getOrigin().y(), 2)+
			pow(targetTranslation.z-currentPosition.getOrigin().z(), 2) > pow(toleranceNextGoal_,2))
			break;
		i++;
	}

    geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint_ = trajectory.points[i].transforms[0];

	//Set the rotation to face the direction which it is heading.
	tf::Quaternion rotation = tf::createIdentityQuaternion();
	double direction = cv::fastAtan2(trajectoryPoint_.translation.y - currentPosition.getOrigin().y(), trajectoryPoint_.translation.x - currentPosition.getOrigin().x()) / 360 * 2 *PI;
	rotation.setRPY((double) 0.0 , (double) 0.0, direction);

	tf::quaternionTFToMsg(rotation, trajectoryPoint_.rotation);

	publishTrajectoryPosition(trajectoryPoint_);*/
}

// Méthode temporaire pour le contrôle par position.
/*void MessageHandler::publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint)
{
  //Convert geometry_msgs::Transform to tf::Transform
  tf::Transform tfTrajectoryPoint;
  tf::transformMsgToTF(trajectoryPoint, tfTrajectoryPoint);

  //Broadcast command
  tf_broadcaster_.sendTransform(tf::StampedTransform(tfTrajectoryPoint, ros::Time::now(), "elikos_arena_origin", "elikos_setpoint"));
}*/