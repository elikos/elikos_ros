/*
* @brief    Ground robot behavior simulation for , nodeth, node(node)e IARC mission 7.
*/

#include <ros/ros.h>


#include <vector>
#include <elikos_lib/pid.hpp>
#include <algorithm>
#include <memory>
#include "TargetRobot.hpp"
#include "ObstacleRobot.hpp"
#include "MAV.h"
#include "GameManager.hpp"

#include "Simulation.hpp"

#include "defines.cpp"

namespace elikos_sim
{
    Simulation::Simulation(int& argc, char** argv, ros::NodeHandle& node): node(node), r(30)
    {

        // ROS initialization
        //ros::init(argc, argv, "robotsim_tf_broadcaster");

        // Parameter initialization
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
        r = ros::Rate(frameRate);

        // MAV initialization
        mav = new elikos_sim::MAV(simSpeed, r.expectedCycleTime());
        mav->setVelXYPID(vel_xy_p, vel_xy_i, vel_xy_d);
        mav->setVelXYMax(vel_xy_max);
        mav->setVelZPID(vel_z_p, vel_z_i, vel_z_d);
        mav->setVelZMax(vel_z_max);

        // Robot & marker initialization


        for (int i = 0; i < nTrgtRobots; ++i) {
            elikos_sim::TargetRobot* robot = new elikos_sim::TargetRobot(i, nTrgtRobots, simSpeed);
            robots.push_back(robot);
            robotMarkers.markers.push_back(robot->getVizMarker());
        }
        for (int i = 0; i < nObsRobots; ++i) {
            elikos_sim::ObstacleRobot* robot = new elikos_sim::ObstacleRobot(i, nObsRobots, simSpeed);
            robots.push_back(robot);
            robotMarkers.markers.push_back(robot->getVizMarker());
        }

        ROS_INFO("Robot markers length: %lu", robotMarkers.markers.size());

        // Publishers
        marker_pub = node.advertise<visualization_msgs::MarkerArray>(TOPIC_NAMES[robotsim_robot_markers], 0); // "robotsim/robot_markers"
        mav_marker_pub = node.advertise<visualization_msgs::Marker>(TOPIC_NAMES[robotsim_mav_marker], 0); // "robotsim/mav_marker"
        setpoint_marker_pub = node.advertise<visualization_msgs::Marker>(TOPIC_NAMES[robotsim_setpoint_marker], 0); // "robotsim/setpoint_marker"
        arena_pub = node.advertise<visualization_msgs::MarkerArray>(TOPIC_NAMES[robotsim_arena_marker], 0); // "robotsim/arena_marker"

        // Subscribers
        pose_sub = node.subscribe(TOPIC_NAMES[mavros_setpoint_local_position], 1000, &elikos_sim::MAV::poseCallback, mav); // "mavros/setpoint/local_position"

        //Arena setup

        setupArenaBoundaries(&arenaMarkers);
    }

    Simulation::~Simulation()
    {

    }

    void Simulation::Exec()
    {

        while(ros::ok()){
            // Receive and set mav setpoints
            ros::spinOnce();
            mav->move();
            mavMarker = mav->getVizMarker();
            setpointMarker = mav ->getSetpointMarker();

            //Collision checking
            for(auto& robot1 : robots)
            {
                handleCollision(robot1, mav);
                for(auto& robot2 : robots)
                    if( robot1 != robot2 && checkCollision(robot1, robot2))
                        robot1->collide();

            }

            robotMarkers.markers.clear();

            //Move the robot
            for(auto robot : robots)
            {
                robot->move(r.expectedCycleTime());
                robotMarkers.markers.push_back(robot->getVizMarker());
                br.sendTransform(tf::StampedTransform(robot->getTransform(), ros::Time::now(), "world", robot->getName()));
            }
            //Remove the robot that are out of bound
            std::remove_if(robots.begin(), robots.end(),Simulation::isOutOfBounds);

            br.sendTransform(tf::StampedTransform(mav->getTransform(), ros::Time::now(), "world", mav->getName()));
            mav_marker_pub.publish(mavMarker);
            setpoint_marker_pub.publish(setpointMarker);
            marker_pub.publish(robotMarkers);
            arena_pub.publish(arenaMarkers);
            r.sleep();
        };
    }

    bool Simulation::handleCollision(elikos_sim::Robot* robot, elikos_sim::MAV* mav)
    {
        tf::Vector3 mavPosition = mav->getTransform().getOrigin();
        tf::Vector3 robotPosition = robot->getTransform().getOrigin();

        double height = mavPosition.getZ();

        mavPosition.setZ(0.0);
        robotPosition.setZ(0.0);

        double distance = mavPosition.distance(robotPosition);
        if (std::abs(distance) < 0.35 && height < 0.1 && height > 0.0)
        {
            robot->interact(true);
            return true;
        }

    }

    bool Simulation::checkCollision(const elikos_sim::Robot* ra, const elikos_sim::Robot* rb){
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

    double Simulation::collisionAngle(tf::Vector3 v, double yaw) {
        double angle = atan2(v.getY(), v.getX()) - yaw;
        if (angle > PI) angle -= 2 * PI;
        else if (angle <= -PI) angle += 2 * PI;
        return fabs(angle);
    }

    bool Simulation::isOutOfBounds(const elikos_sim::Robot* robot){
        bool result = false;

        if(robot->getTransform().getOrigin().getY() > 10){
            //green line
            result = true;
            ROS_INFO("Green line crossed");
            //notify manager green line
        } else if(robot->getTransform().getOrigin().getY() < -10){
            //red line
            result = true;
            ROS_INFO("Red line crossed");
            //notify manager red line
        } else if(robot->getTransform().getOrigin().getX() < -10){
            //white line -10
            result = true;
            ROS_INFO("White line 01 crossed");
            //notify manager white line 1
        } else if(robot->getTransform().getOrigin().getX() > 10){
            //white line +10
            result = true;
            ROS_INFO("White line 02 crossed");
            //notify manager white line 2
        }

        return result;
    }

    void Simulation::setupArenaBoundaries(visualization_msgs::MarkerArray* arenaMarkers){
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

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotsim_tf_broadcaster");
    ros::NodeHandle node;
    elikos_sim::Simulation sim(argc, argv, node);

    sim.Exec();

    return 0;
}
