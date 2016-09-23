
#include "moveit_move_group.h"

Moveit_move_group::Moveit_move_group():
  parent_frame_("elikos_arena_origin"),
  child_frame_("elikos_setpoint"),
  toleranceAchieveGoal_(0.5),
  toleranceNextGoal_(0.5),
  toleranceFreeOctomap_(0.1),
  safetyTime_(3.0)
{
  //move_group settings
  group_.setPlanningTime(1.0);//In seconds
  //group_.setGoalTolerance (0.1);//In meters
  //The workspace represents the boundaries of the planning volume.
  group_.setWorkspace(-10,-10,0,10,10,3);

  group_.allowReplanning(true);

  group_.setNumPlanningAttempts(20);
}

Moveit_move_group::~Moveit_move_group()
{
}

void Moveit_move_group::move(geometry_msgs::PoseStamped target)
{

  std::vector<double> quad_variable_values;

  group_.getCurrentState()->copyJointGroupPositions(group_.getCurrentState()->getRobotModel()->getJointModelGroup(group_.getName()), quad_variable_values);

  //This is the new position and orientation for the quadcopter
  quad_variable_values[0] = target.pose.position.x;//position.x
  quad_variable_values[1] = target.pose.position.y;//position y
  quad_variable_values[2] = target.pose.position.z;//position z
  quad_variable_values[3] = 0;//target.pose.orientation.x;//quaternion x
  quad_variable_values[4] = 0;//target.pose.orientation.y;//quaternion y
  quad_variable_values[5] = 0;//target.pose.orientation.z;//quaternion z
  quad_variable_values[6] = 1;//target.pose.orientation.w;//quaternion w

  //Plan the goal pose
  group_.setJointValueTarget(quad_variable_values);

  //Plan and execute the trajectory
  //group_.asyncMove();
  moveit::planning_interface::MoveGroup::Plan plan;

  try
  {
    tf::StampedTransform currentPosition;
    listener.lookupTransform(parent_frame_, "elikos_base_link",
                              ros::Time(0), currentPosition);
    if(target.pose.position.z == -1.0)
    {
      trajectoryPoint_.translation.x = target.pose.position.x;
      trajectoryPoint_.translation.y = target.pose.position.y;
      trajectoryPoint_.translation.z = target.pose.position.z;

      //Set the rotation to the current one.
      tf::quaternionTFToMsg(currentPosition.getRotation(), trajectoryPoint_.rotation);

      publishTrajectoryPoint(trajectoryPoint_);
    }
    else if( pow(target.pose.position.x-currentPosition.getOrigin().x(), 2)+
        pow(target.pose.position.y-currentPosition.getOrigin().y(), 2)+
        pow(target.pose.position.z-currentPosition.getOrigin().z(), 2) > pow(toleranceAchieveGoal_, 2))
    {

      group_.setStartStateToCurrentState();
      moveit_msgs::MoveItErrorCodes err = group_.plan(plan);

      if(err.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_ERROR_STREAM("New trajectory!");
        //Execute first point in trajectory.
        trajectory_msgs::MultiDOFJointTrajectory trajectoryToExecute = plan.trajectory_.multi_dof_joint_trajectory;
        listener.lookupTransform(parent_frame_, "elikos_base_link",
                                ros::Time(0), currentPosition);
        int i = 0;
        while(i < trajectoryToExecute.points.size()-1)
        {
            geometry_msgs::Vector3 targetTranslation = trajectoryToExecute.points[i].transforms[0].translation;
            if(pow(targetTranslation.x-currentPosition.getOrigin().x(), 2)+
                pow(targetTranslation.y-currentPosition.getOrigin().y(), 2)+
                pow(targetTranslation.z-currentPosition.getOrigin().z(), 2) > pow(toleranceNextGoal_,2))
              break;
            i++;
        }
        trajectoryPoint_ = trajectoryToExecute.points[i].transforms[0];

        //Set the rotation to face the direction which it is heading.
        tf::Quaternion rotation = tf::createIdentityQuaternion();
        double direction = cv::fastAtan2(trajectoryPoint_.translation.y - currentPosition.getOrigin().y(), trajectoryPoint_.translation.x - currentPosition.getOrigin().x()) / 360 * 2 *PI;
        rotation.setRPY((double) 0.0 , (double) 0.0, direction);

        tf::quaternionTFToMsg(rotation, trajectoryPoint_.rotation);

        publishTrajectoryPoint(trajectoryPoint_);
      }
      else
      {
        ROS_ERROR_STREAM("Safety mode!");
        trajectoryPoint_.translation.x = currentPosition.getOrigin().x();
        trajectoryPoint_.translation.y = currentPosition.getOrigin().y();
        trajectoryPoint_.translation.z = 2.0;

        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        ros::service::call("/clear_octomap", req, res);

        tf::quaternionTFToMsg(currentPosition.getRotation(), trajectoryPoint_.rotation);

        for(int j=0; j<safetyTime_/10; j++)
        {
          publishTrajectoryPoint(trajectoryPoint_);
          sleep(0.1);
        }
      }
    }

  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }
}

void Moveit_move_group::publishTrajectoryPoint(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint)
{
  //Convert geometry_msgs::Transform to tf::Transform
  tf::Transform tfTrajectoryPoint;
  tf::transformMsgToTF(trajectoryPoint, tfTrajectoryPoint);

  //Broadcast command
  tf_broadcaster_.sendTransform(tf::StampedTransform(tfTrajectoryPoint, ros::Time::now(), parent_frame_, child_frame_));
}
