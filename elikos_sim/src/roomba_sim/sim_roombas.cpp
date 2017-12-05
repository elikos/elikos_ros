/**
 * \file sim_roombas.cpp
 * \brief Simulation manager for roomba_sim. Creates and handles robots and quad.
 * \author christophebedard
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include "elikos_roomba/groundrobot.h"
#include "elikos_roomba/obstaclerobot.h"
#include "Quad.h"

static const double LOOP_RATE_SIM = 10.0; /**< loop rate */
static const std::string NODE_NAME = "sim_roombas"; /**< node name */

static const double TARGET_ROBOT_RADIUS = 1.0; /**< circle radius for target robot initial position */
static const double OBSTACLE_ROBOT_RADIUS = 5.0; /**<  circle radius for obstacle robot initial position */

std::vector<Robot*> robots;
Quad* quad;

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::Rate rate(LOOP_RATE_SIM);

    ros::NodeHandle n_p("~");
    int nbTargetRobots, nbObstacleRobots;
    n_p.getParam("/target_robot_count", nbTargetRobots);
    n_p.getParam("/obstacle_robot_count", nbObstacleRobots);

    // create target robots
    for (int i = 0; i < nbTargetRobots; ++i) {
        double pos_x = TARGET_ROBOT_RADIUS * cos(i * (360 / nbTargetRobots) * DEG_TO_RAD);
        double pos_y = TARGET_ROBOT_RADIUS * sin(i * (360 / nbTargetRobots) * DEG_TO_RAD);
        double yaw = i * (360 / nbTargetRobots) * DEG_TO_RAD;
        std::string color = (i%2 == 0) ? "red" : "green";
        robots.push_back(new GroundRobot(n, i+1, tf::Vector3(pos_x, pos_y, 0.0), yaw, color));
    }

    // create obstacle robots
    std::vector<std::string> heights = {"05", "10", "15", "20"};
    for (int i = 0; i < nbObstacleRobots; ++i) {
        double pos_x = OBSTACLE_ROBOT_RADIUS * cos(i * (360 / nbObstacleRobots) * DEG_TO_RAD);
        double pos_y = OBSTACLE_ROBOT_RADIUS * sin(i * (360 / nbObstacleRobots) * DEG_TO_RAD);
        double yaw = (-90.0 + (i * (360 / nbObstacleRobots))) * DEG_TO_RAD;
        std::string height = heights[i%4];
        robots.push_back(new ObstacleRobot(n, i+1, tf::Vector3(pos_x, pos_y, 0.0), yaw, height));
    }

    // create quad
    quad = new Quad(n, rate.expectedCycleTime());

    // run simulation
    while (ros::ok())
    {
        // quad
        quad->update();

        // robots
        for (auto& robotBase : robots) {
            robotBase->update();

            std::string baseNS = robotBase->getNamespace();

            // if target robot
            if (robotBase->getRobotType() == "ground") {
                // check robot bumper collisions
                for (auto& robot : robots) {
                    std::string robotNS = robot->getNamespace();
                    // if not same robot
                    if (baseNS != robotNS) {
                        robotBase->checkRobotCollision(robot->getPosition());
                    }
                }

                // check top interaction
                robotBase->checkTopInteraction(quad->getPosition(), quad->getInteractionDiameter());
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}