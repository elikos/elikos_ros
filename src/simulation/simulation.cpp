/**
* @brief    Ground robot behavior simulation for the IARC mission 7.
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include "MAV.h"
#include <vector>
#include <elikos_lib/pid.hpp>
#include "Robot.hpp"
#include "TargetRobot.hpp"
#include "ObstacleRobot.hpp"

#ifndef PI
#define PI 3.14159265
#endif

bool checkCollision(Robot* ra, Robot* rb);

double collisionAngle(tf::Vector3 v, double yaw);

void setVector(tf::Vector3 &v, double x, double y, double z);

int main(int argc, char **argv) {
    // ROS initialization
    ros::init(argc, argv, "robotsim_tf_broadcaster");
    ros::NodeHandle node;

    // Parameter initialization
    double simSpeed;
    int nTrgtRobots, nObsRobots, frameRate;
    node.param<double>("simulation_speed", simSpeed, 1.0);
    node.param<int>("target_robot_count", nTrgtRobots, 10);
    node.param<int>("obstacle_robot_count", nObsRobots, 4);
    node.param<int>("frame_rate", frameRate, 30);
    int totalRobots = nTrgtRobots + nObsRobots;


    // Robot & marker initialization
    std::vector<Robot *> robots;
    visualization_msgs::MarkerArray robotMarkers;
    for (int i = 0; i < nTrgtRobots; ++i) {
        robots.push_back(new TargetRobot(i, nTrgtRobots, simSpeed));
        robotMarkers.markers.push_back(robots.back()->getVizMarker());
    }
    for (int i = 0; i < nObsRobots; ++i) {
        robots.push_back(new ObstacleRobot(i, nObsRobots, simSpeed));
        robotMarkers.markers.push_back(robots.back()->getVizMarker());
    }

    ROS_INFO("Robot markers legnth: %lu", robotMarkers.markers.size());


    // Publishers
    ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("robotsim/robot_markers", 0);
    tf::TransformBroadcaster br;

    // Loop rate (Hz)
    ros::Rate r(frameRate);

    while(ros::ok()){
        //Collision checking
        for(std::vector<Robot*>::iterator it = robots.begin(); it != robots.end(); ++it){
            for(std::vector<Robot*>::iterator it2 = robots.begin(); it2 != robots.end(); ++it2){
                if(*it != *it2){
                    if(checkCollision(*it, *it2)){
                        (*it)->collide();
                    }
                }
            }
        }

        std::vector<visualization_msgs::Marker>::iterator mit = robotMarkers.markers.begin();
        for(std::vector<Robot*>::iterator it = robots.begin(); it != robots.end(); ++it){
            (*it)->move(r.expectedCycleTime());
            (*mit) = (*it)->getVizMarker();
            ++mit;
            br.sendTransform(tf::StampedTransform((*it)->getTransform(), ros::Time::now(), "world", (*it)->getName()));
        }
        marker_pub.publish(robotMarkers);
        r.sleep();
    }

//
//    // Robot & marker initialization
//    std::vector <GroundRobot *> robots;
//    robots.reserve(totalRobots);
//    visualization_msgs::MarkerArray robotMarkers;
//    for (int i = 0; i < nTrgtRobots; i++) {
//        robots.push_back(new GroundRobot(TARGET_ROBOT, nTrgtRobots, i, simSpeed));
//        robotMarkers.markers.push_back(getRobotMarker(robots[i]));
//    }
//    for (int i = 0; i < nObsRobots; i++) {
//        robots.push_back(new GroundRobot(OBSTACLE_ROBOT_RND, nObsRobots, i, simSpeed));
//        robotMarkers.markers.push_back(getRobotMarker(robots[i]));
//    }
//
//    // Publishers
//    ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("robotsim/robot_markers", 0);
//    tf::TransformBroadcaster br;
//
//    // Loop rate (Hz)
//    ros::Rate r(frameRate);
//
//    // Main Loop
//    while (ros::ok()) {
//        // Collision checking
//        for (int i = 0; i < (totalRobots); i++) {
//            for (int j = 0; j < totalRobots; j++) {
//                if (i == j) continue;
//                if (checkCollision(robots[i], robots[j])) {
//                    robots[i]->collide();
//                }
//            }
//        }
//        // Advance robots to next frame and publish tf & marker
//        for (int i = 0; i < totalRobots; i++) {
//            robots[i]->advance(r.expectedCycleTime());
//            robotMarkers.markers[i] = getRobotMarker(robots[i]);
//            br.sendTransform(tf::StampedTransform(robots[i]->getTransform(), ros::Time::now(), "world", robots[i]->getName()));
//        }
//        marker_pub.publish(robotMarkers);
//        r.sleep();
//    }
    return 0;
};

bool checkCollision(Robot* ra, Robot* rb){
    tf::Vector3 vA = ra->getTransform().getOrigin();
    tf::Vector3 vB = rb->getTransform().getOrigin();
    tf::Vector3 xAxis;
    xAxis.setX(1);
    xAxis.setY(0);
    xAxis.setZ(0);
    tf::Quaternion qA = ra->getTransform().getRotation();

    if (vA.distance(vB) > 0.35) {
        return false;
    } else if (collisionAngle(vB - vA, tf::getYaw(qA)) > (4 * PI / 9)) {
        return false;
    } else {
        return true;
    }
}

double collisionAngle(tf::Vector3 v, double yaw) {
    double angle = atan2(v.getY(), v.getX()) - yaw;
    if (angle > PI) angle -= 2 * PI;
    else if (angle <= -PI) angle += 2 * PI;
    return fabs(angle);
}

void setVector(tf::Vector3 &v, double x, double y, double z) {
    v.setX(x);
    v.setY(y);
    v.setZ(z);
}
