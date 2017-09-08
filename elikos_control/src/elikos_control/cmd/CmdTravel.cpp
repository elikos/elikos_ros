#include "CmdTravel.h"

CmdTravel::CmdTravel(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    cmdPriority_ = PriorityLevel::ALWAYS_ABORTABLE;
	cmdCode_ = CmdCode::MOVE_TO_POINT;
	
	nh_->getParam("/elikos_ai/dimension_c", dimension_c_);
 	nh_->getParam("/elikos_ai/max_altitude", max_altitude_);
    double takeoff_altitude = 1;
    nh_->getParam("/elikos_ai/takeoff_altitude", takeoff_altitude);
	lastPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, takeoff_altitude }));
    lastPosition_.child_frame_id_ = SETPOINT;
    lastPosition_.frame_id_ = WORLD_FRAME;

	threshold_ = 0.8;
	nh_->getParam("/elikos_ai/min_step", threshold_);
	max_step_ = 2;
	nh_->getParam("/elikos_ai/max_step", max_step_);
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
  tf_broadcaster_.sendTransform(tf::StampedTransform(tfTrajectoryPoint, ros::Time::now(), WORLD_FRAME, SETPOINT));
}

void CmdTravel::execute()
{
    std_msgs::String msg;
    msg.data = "Travel";
    statePubCommand_.publish(msg);

    isAborted_ = false;
	ros::Rate rate(30.0);
    
	stepInTrajectory_ = 0;

	while(!isAborted_ && stepInTrajectory_ < cmdTrajectory_.points.size()-1)
	{
        ros::spinOnce();
		tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
		double step_lenght = 0;
		bool is_too_near = true;
		bool is_outside_boundaries = false;

		geometry_msgs::Vector3 dst = cmdTrajectory_.points[cmdTrajectory_.points.size() - 1].transforms[0].translation;

		while(is_too_near && !is_outside_boundaries && stepInTrajectory_ < cmdTrajectory_.points.size())
		{
			geometry_msgs::Vector3 targetTranslation = cmdTrajectory_.points[stepInTrajectory_].transforms[0].translation;
			if ( targetTranslation.z > max_altitude_ ||
				std::abs(targetTranslation.x) > dimension_c_ / 2.0 ||
				std::abs(targetTranslation.y) > dimension_c_ / 2.0 )
				{
					is_outside_boundaries = true;
				}

			step_lenght = pow(targetTranslation.x-lastPosition_.getOrigin().x(), 2)+
				pow(targetTranslation.y-lastPosition_.getOrigin().y(), 2)+
				pow(targetTranslation.z-lastPosition_.getOrigin().z(), 2);
			if(step_lenght > pow(threshold_,2))
			{
				is_too_near = false;
			}
			else
			{
				if ((stepInTrajectory_ == cmdTrajectory_.points.size() - 1) || is_outside_boundaries)
				{
					break;
				}
				else
				{
					stepInTrajectory_++;
				}
			}
		}
		trajectoryPoint_ = cmdTrajectory_.points[stepInTrajectory_].transforms[0];

		if(step_lenght > pow(max_step_,2))
		{
			// Interpolation
			double direction = cv::fastAtan2(trajectoryPoint_.translation.y - lastPosition_.getOrigin().y(), trajectoryPoint_.translation.x - lastPosition_.getOrigin().x()) / 360 * 2 *PI;
			trajectoryPoint_.translation.x = lastPosition_.getOrigin().x() + max_step_ * std::cos(direction);
			trajectoryPoint_.translation.y = lastPosition_.getOrigin().y() + max_step_ * std::sin(direction);
		}

		tf::Quaternion identity_rotation = tf::createIdentityQuaternion();
		tf::quaternionTFToMsg(identity_rotation, trajectoryPoint_.rotation);

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
