#include "CmdTravel.h"

CmdTravel::CmdTravel(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    cmdPriority_ = PriorityLevel::ALWAYS_ABORTABLE;
	cmdCode_ = 4;
	
	lastPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 2.0 }));
    lastPosition_.child_frame_id_ = "elikos_setpoint";
    lastPosition_.frame_id_ = WORLD_FRAME;
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
    isAborted_ = false;
	ros::Rate rate(30.0);
    
	tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
	stepInTrajectory_ = 0;

	while(!isAborted_)
	{

		while(stepInTrajectory_ < cmdTrajectory_.points.size()-1)
		{
			geometry_msgs::Vector3 targetTranslation = cmdTrajectory_.points[stepInTrajectory_].transforms[0].translation;
			if(pow(targetTranslation.x-lastPosition_.getOrigin().x(), 2)+
				pow(targetTranslation.y-lastPosition_.getOrigin().y(), 2)+
				pow(targetTranslation.z-lastPosition_.getOrigin().z(), 2) > pow(THRESHOLD,2))
				break;
			stepInTrajectory_++;
		}

		trajectoryPoint_ = cmdTrajectory_.points[stepInTrajectory_].transforms[0];

		//Set the rotation to face the direction which it is heading.
		tf::Quaternion rotation = tf::createIdentityQuaternion();
		double direction = cv::fastAtan2(trajectoryPoint_.translation.y - lastPosition_.getOrigin().y(), trajectoryPoint_.translation.x - lastPosition_.getOrigin().x()) / 360 * 2 *PI;
		rotation.setRPY((double) 0.0 , (double) 0.0, direction);

		tf::quaternionTFToMsg(rotation, trajectoryPoint_.rotation);

		publishTrajectoryPosition(trajectoryPoint_);
	}
}


void CmdTravel::abort()
{
	isAborted_ = true;
}

void CmdTravel::ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory)
{
	CmdAbs::setTrajectory(trajectory);


}
