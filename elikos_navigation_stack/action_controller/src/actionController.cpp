#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

//Large parts of the code come from : https://github.com/AlessioTonioni/Autonomous-Flight-ROS
class Controller{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	Controller(ros::NodeHandle &n) :
		node_(n),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
				false),
				has_active_goal_(false),
				parent_frame_("elikos_base_link"),
				child_frame_("elikos_setpoint")
{
		creato=0;
		empty.linear.x=0;
		empty.linear.y=0;
		empty.linear.z=0;
		empty.angular.z=0;
		empty.angular.y=0;
		empty.angular.x=0;
		action_server_.start();
		ROS_INFO_STREAM("Node ready!");
}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	tf::TransformBroadcaster tf_broadcaster_;
	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
	geometry_msgs::Twist cmd;
	pthread_t trajectoryExecutor;
	int creato;
	std::string parent_frame_;
	std::string child_frame_;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > trajectoryToExecute;

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(creato){
				ROS_INFO_STREAM("Stop thread");
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(creato){
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		trajectoryToExecute = gh.getGoal()->trajectory;

		//controllore solo per il giunto virtuale Base
		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			creato=1;
			ROS_INFO_STREAM("Thread for trajectory execution created");
		} else {
			ROS_INFO_STREAM("Thread creation failed!");
		}

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void executeTrajectory(){
		if(trajectoryToExecute.joint_names[0]=="virtual_joint" && trajectoryToExecute.points.size()>0)
		{
			//Set start position
			geometry_msgs::Transform_<std::allocator<void> > trajectoryStartPoint = trajectoryToExecute.points[0].transforms[0];
			lastPosition.translation=trajectoryStartPoint.translation;
			lastPosition.rotation=trajectoryStartPoint.rotation;

			//For now we only exec the first transform of the trajectory.
			for(int k=1; k < 2 && k < trajectoryToExecute.points.size(); k++){

				geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint = trajectoryToExecute.points[k].transforms[0];

				publishCommand(trajectoryPoint);

				//update start position
				lastPosition.translation=trajectoryPoint.translation;
				lastPosition.rotation=trajectoryPoint.rotation;
			}
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		creato=0;
	}
	void publishCommand(geometry_msgs::Transform_<std::allocator<void> > trajectoryPoint)
	{
		//Convert geometry_msgs::Transform to tf::Transform
		tf::Transform tfTrajectoryPoint;
    tf::transformMsgToTF(trajectoryPoint, tfTrajectoryPoint);

		//Broadcast command
		tf_broadcaster_.sendTransform(tf::StampedTransform(tfTrajectoryPoint, ros::Time::now(), parent_frame_, child_frame_));
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_controller_node");
	ros::NodeHandle node;//("~");
	Controller control(node);

	ros::spin();

	return 0;
}
