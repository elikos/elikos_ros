#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include "elikos_roomba/groundrobot.h"
#include "elikos_roomba/obstaclerobot.h"

static const double LOOP_RATE_SIM = 10.0;
static const std::string NODE_NAME = "sim_roombas";
static const std::string TF_MIDDLE_ARENA = "/elikos_arena_origin";
static const std::string ARENA_SQUARE_MODEL = "package://elikos_roomba/models/arena_square.dae";

static const double TARGET_ROBOT_RADIUS = 1.0;
static const double OBSTACLE_ROBOT_RADIUS = 5.0;

std::vector<Robot*> robots;


int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

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

    ros::Rate rate(LOOP_RATE_SIM);
    while (ros::ok())
    {
        for (auto& robot : robots) {
            robot->update();
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}