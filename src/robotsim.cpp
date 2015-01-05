/**
* @brief    Ground robot behavior simulation for the IARC mission 7.
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <elikos_ros/GroundRobot.h>
#include <vector>
using std::vector;

bool checkCollision(GroundRobot robotA, GroundRobot robotB);
double collisionAngle(tf::Vector3 v, double yaw);
void setVector(tf::Vector3& v, double x, double y, double z);

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
		// Collision checking
		for (int i = 0; i < 10; i++){
			for (int j = 0; j < 10; j++){
				if (i==j) continue;
				if (checkCollision(robot[i], robot[j])){
					robot[i].collide();
				}
			}
		}
		// Advance robots to next frame and publish tf
		for (int i = 0; i < 10; i++){
			robot[i].advance(r.expectedCycleTime());
			br.sendTransform(tf::StampedTransform(robot[i].getTransform(), ros::Time::now(), "world", robot[i].getName()));
		}
		r.sleep();
	}
	return 0;
};

bool checkCollision(GroundRobot robotA, GroundRobot robotB){
	tf::Vector3 vA = robotA.getTransform().getOrigin();
	tf::Vector3 vB = robotB.getTransform().getOrigin();
	tf::Vector3 xAxis;
	setVector(xAxis, 1, 0, 0);
	tf::Quaternion qA = robotA.getTransform().getRotation();
//	if (robotA.getName() == "robot0" && robotB.getName() == "robot1")
//		std::cout << "Collision vector angle: " << atan2((vB-vA).getY(), (vB-vA).getX()) << "\tYaw:" << tf::getYaw(qA) << "\tcollisionAngle:" << collisionAngle(vB-vA, tf::getYaw(qA)) << "\n";

	if (vA.distance(vB) > 0.35) {
		return false;
	} else if (collisionAngle(vB-vA, tf::getYaw(qA)) > (4*PI/9)) {
		return false;
	} else {
		return true;
	}
}

double collisionAngle(tf::Vector3 v, double yaw){
	double angle = atan2(v.getY(), v.getX())-yaw;
	if (angle > PI) angle-=2*PI;
	else if (angle <= -PI) angle+=2*PI;
	return fabs(angle);
}

void setVector(tf::Vector3& v, double x, double y, double z){
	v.setX(x);
	v.setY(y);
	v.setZ(z);
}
