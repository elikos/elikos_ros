#include "CmdTravel.h"

CmdTravel::CmdTravel(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 2.0 }));
    targetPosition_.child_frame_id_ = MAV_FRAME;
    targetPosition_.frame_id_ = WORLD_FRAME;
}

CmdTravel::~CmdTravel()
{
}

void CmdTravel::publishTrajectoryPosition(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint)
{
  //Convert geometry_msgs::Transform to tf::Transform
  tf::Transform tfTrajectoryPoint;
  tf::transformMsgToTF(trajectoryPoint, tfTrajectoryPoint);

  //Broadcast command
  tf_broadcaster_.sendTransform(tf::StampedTransform(tfTrajectoryPoint, ros::Time::now(), WORLD_FRAME, "elikos_setpoint"));
}

void CmdTravel::execute()
{
    ros::Rate rate(30.0);
    
	tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
	int i = 0;
	while(i < cmdTrajectory_.points.size()-1)
	{
		geometry_msgs::Vector3 targetTranslation = cmdTrajectory_.points[i].transforms[0].translation;
        double distance = lastPosition_.getOrigin().distance(targetTranslation.getOrigin());
		if (distance > THRESHOLD)
            break;
		i++;
	}

    geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint_ = cmdTrajectory_.points[i].transforms[0];

	//Set the rotation to face the direction which it is heading.
	tf::Quaternion rotation = tf::createIdentityQuaternion();
	double direction = cv::fastAtan2(trajectoryPoint_.translation.y - currentPosition.getOrigin().y(), trajectoryPoint_.translation.x - currentPosition.getOrigin().x()) / 360 * 2 *PI;
	rotation.setRPY((double) 0.0 , (double) 0.0, direction);

	tf::quaternionTFToMsg(rotation, trajectoryPoint_.rotation);

	publishTrajectoryPosition(trajectoryPoint_);

    
}


void CmdTravel::abort()
{
}

void CmdTravel::ajustement()
{
}
