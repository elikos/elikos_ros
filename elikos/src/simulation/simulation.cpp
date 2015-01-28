/**
* @brief    Ground robot behavior simulation for the IARC mission 7.
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <elikos_lib/pid.hpp>
#include "Robot.hpp"
#include "TargetRobot.hpp"
#include "ObstacleRobot.hpp"
#include "MAV.h"

#ifndef PI
#define PI 3.14159265
#endif

bool checkCollision(Robot* ra, Robot* rb);

double collisionAngle(tf::Vector3 v, double yaw);

void setupArenaBoundaries(visualization_msgs::MarkerArray* arenaMarkers);

int main(int argc, char **argv) {
    // ROS initialization
    ros::init(argc, argv, "robotsim_tf_broadcaster");
    ros::NodeHandle node;

    // Parameter initialization
    double simSpeed, vel_xy_p, vel_xy_i, vel_xy_d, vel_z_p, vel_z_i, vel_z_d;
    double vel_xy_max, vel_z_max;
    int nTrgtRobots, nObsRobots, frameRate;
    node.param<double>("simulation_speed", simSpeed, 1.0);
    node.param<int>("target_robot_count", nTrgtRobots, 10);
    node.param<int>("obstacle_robot_count", nObsRobots, 4);
    node.param<int>("frame_rate", frameRate, 30);

    // PIDs & max velocities
    node.param<double>("vel_xy_p", vel_xy_p, 2.0);
    node.param<double>("vel_xy_i", vel_xy_i, 0.5);
    node.param<double>("vel_xy_d", vel_xy_d, 1.0);
    node.param<double>("vel_xy_max", vel_xy_max, 1.0);
    node.param<double>("vel_z_p", vel_z_p, 2.0);
    node.param<double>("vel_z_i", vel_z_i, 0.5);
    node.param<double>("vel_z_d", vel_z_d, 1.0);
    node.param<double>("vel_z_max", vel_z_max, 0.5);
    int totalRobots = nTrgtRobots + nObsRobots;

    // Loop rate (Hz)
    ros::Rate r(frameRate);

    // MAV initialization
    MAV * mav = new MAV(simSpeed, r.expectedCycleTime());
    mav->setVelXYPID(vel_xy_p, vel_xy_i, vel_xy_d);
    mav->setVelXYMax(vel_xy_max);
    mav->setVelZPID(vel_z_p, vel_z_i, vel_z_d);
    mav->setVelZMax(vel_z_max);
    visualization_msgs::Marker mavMarker;
    visualization_msgs::Marker setpointMarker;

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

    ROS_INFO("Robot markers length: %lu", robotMarkers.markers.size());

    // Publishers
    ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("robotsim/robot_markers", 0);
    ros::Publisher mav_marker_pub = node.advertise<visualization_msgs::Marker>("robotsim/mav_marker", 0);
    ros::Publisher setpoint_marker_pub = node.advertise<visualization_msgs::Marker>("robotsim/setpoint_marker", 0);
    ros::Publisher arena_pub = node.advertise<visualization_msgs::MarkerArray>("robotsim/arena_marker", 0);
    tf::TransformBroadcaster br;

    // Subscribers
    ros::Subscriber pose_sub = node.subscribe("mavros/setpoint/local_position", 1000, &MAV::poseCallback, mav);

    //Arena setup
    visualization_msgs::MarkerArray arenaMarkers;
    setupArenaBoundaries(&arenaMarkers);

    while(ros::ok()){
        // Receive and set mav setpoints
        ros::spinOnce();
        mav->move();
        mavMarker = mav->getVizMarker();
        setpointMarker = mav ->getSetpointMarker();

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
        br.sendTransform(tf::StampedTransform(mav->getTransform(), ros::Time::now(), "world", mav->getName()));
        mav_marker_pub.publish(mavMarker);
        setpoint_marker_pub.publish(setpointMarker);
        marker_pub.publish(robotMarkers);
        arena_pub.publish(arenaMarkers);
        r.sleep();
    }

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

void setupArenaBoundaries(visualization_msgs::MarkerArray* arenaMarkers){
    visualization_msgs::Marker greenLine, redLine, whiteLine1, whiteLine2;
    greenLine.header.frame_id = "world";
    greenLine.header.stamp = ros::Time();
    greenLine.lifetime = ros::Duration();
    greenLine.ns = "greenLine";
    greenLine.id = 0;
    greenLine.type = visualization_msgs::Marker::CUBE;
    greenLine.action = visualization_msgs::Marker::ADD;
    greenLine.scale.x = 20;
    greenLine.scale.y = 0.1;
    greenLine.scale.z = 0.001;
    greenLine.pose.position.x = 0;
    greenLine.pose.position.y = 10;
    greenLine.pose.position.z = 0;
    greenLine.pose.orientation.x = 0.0;
    greenLine.pose.orientation.y = 0.0;
    greenLine.pose.orientation.z = 0.0;
    greenLine.pose.orientation.w = 0.0;
    greenLine.color.a = 1.0;
    greenLine.color.r = 0.0;
    greenLine.color.g = 1.0;
    greenLine.color.b = 0.0;

    redLine.header.frame_id = "world";
    redLine.header.stamp = ros::Time();
    redLine.lifetime = ros::Duration();
    redLine.ns = "redLine";
    redLine.id = 0;
    redLine.type = visualization_msgs::Marker::CUBE;
    redLine.action = visualization_msgs::Marker::ADD;
    redLine.scale.x = 20;
    redLine.scale.y = 0.1;
    redLine.scale.z = 0.001;
    redLine.pose.position.x = 0;
    redLine.pose.position.y = -10;
    redLine.pose.position.z = 0;
    redLine.pose.orientation.x = 0.0;
    redLine.pose.orientation.y = 0.0;
    redLine.pose.orientation.z = 0.0;
    redLine.pose.orientation.w = 0.0;
    redLine.color.a = 1.0;
    redLine.color.r = 1.0;
    redLine.color.g = 0.0;
    redLine.color.b = 0.0;

    whiteLine1.header.frame_id = "world";
    whiteLine1.header.stamp = ros::Time();
    whiteLine1.lifetime = ros::Duration();
    whiteLine1.ns = "whiteLine";
    whiteLine1.id = 0;
    whiteLine1.type = visualization_msgs::Marker::CUBE;
    whiteLine1.action = visualization_msgs::Marker::ADD;
    whiteLine1.scale.x = 0.1;
    whiteLine1.scale.y = 20;
    whiteLine1.scale.z = 0.001;
    whiteLine1.pose.position.x = -10;
    whiteLine1.pose.position.y = 0;
    whiteLine1.pose.position.z = 0;
    whiteLine1.pose.orientation.x = 0.0;
    whiteLine1.pose.orientation.y = 0.0;
    whiteLine1.pose.orientation.z = 0.0;
    whiteLine1.pose.orientation.w = 0.0;
    whiteLine1.color.a = 1.0;
    whiteLine1.color.r = 1.0;
    whiteLine1.color.g = 1.0;
    whiteLine1.color.b = 1.0;

    whiteLine2.header.frame_id = "world";
    whiteLine2.header.stamp = ros::Time();
    whiteLine2.lifetime = ros::Duration();
    whiteLine2.ns = "whiteLine";
    whiteLine2.id = 1;
    whiteLine2.type = visualization_msgs::Marker::CUBE;
    whiteLine2.action = visualization_msgs::Marker::ADD;
    whiteLine2.scale.x = 0.1;
    whiteLine2.scale.y = 20;
    whiteLine2.scale.z = 0.001;
    whiteLine2.pose.position.x = 10;
    whiteLine2.pose.position.y = 0;
    whiteLine2.pose.position.z = 0;
    whiteLine2.pose.orientation.x = 0.0;
    whiteLine2.pose.orientation.y = 0.0;
    whiteLine2.pose.orientation.z = 0.0;
    whiteLine2.pose.orientation.w = 0.0;
    whiteLine2.color.a = 1.0;
    whiteLine2.color.r = 1.0;
    whiteLine2.color.g = 1.0;
    whiteLine2.color.b = 1.0;

    arenaMarkers->markers.push_back(greenLine);
    arenaMarkers->markers.push_back(redLine);
    arenaMarkers->markers.push_back(whiteLine1);
    arenaMarkers->markers.push_back(whiteLine2);
}
