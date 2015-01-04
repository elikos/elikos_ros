/**
* @brief    Ground robot behavior simulation for the IARC mission 7.
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <elikos_ros/GroundRobot.h>
#include <vector>
using std::vector;

int main(int argc, char** argv){
	const int freq = 30;
	ros::init(argc, argv, "robotsim_tf_broadcaster");
	ros::NodeHandle node;
	ros::Rate r(freq);
	std::vector<GroundRobot> robot;
	tf::TransformBroadcaster br;

	robot.reserve(10);
	for (int i = 0; i < 10; i++){
		robot.push_back(GroundRobot(i));
	}

	while(ros::ok()) {
		for (int i = 0; i < 10; i++){
			robot[i].advance(r.expectedCycleTime());
			br.sendTransform(tf::StampedTransform(robot[i].getTransform(), ros::Time::now(), "world", robot[i].getName()));
		}
		r.sleep();
	}
	return 0;
};

