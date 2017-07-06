#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>  //for using the function sleep
#include <unordered_map>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "robot.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

#define DEBUG 1

#define RED 0
#define GREEN 1
const int NUM_ROBOTS_PER_COLOR = 1;

std::unordered_map<int, cv::Vec3b> map_id_color;

std::vector<Robot> redRobots(NUM_ROBOTS_PER_COLOR);
std::vector<Robot> greenRobots(NUM_ROBOTS_PER_COLOR);

int DoMatch(geometry_msgs::Point inputPoint, std::vector<Robot>& robots) {
    double dist = DBL_MAX;
    // TODO: do smtgh like this:
    /*
    std::vector<type>::iterator iter = std::find_if(vec.begin(), vec.end(),
    comparisonFunc);
      size_t index = std::distance(vec.begin(), iter);
      if(index == vec.size())
     {
       //invalid
     }
     */
    int closestRobotIndex = robots.size();
    for (int i = 0; i < robots.size(); i++) {
        if (!robots.at(i).isNew) {
            double robotDist = robots.at(i).getDistanceFrom(inputPoint);

            // On prend le robot le plus proche de la inputPose
            if (robotDist < dist) {
                // TODO: prendre en compte l'incertitude?
                dist = robotDist;
                closestRobotIndex = i;
            }
        }
    }

    if (closestRobotIndex) {
        // Si tous les robots sont nouveaux, on assigne au premier disponible
        for (int i = 0; i < robots.size(); i++)
            if (robots.at(i).isNew) {
                robots.at(i).isNew = false;
                closestRobotIndex = i;
                break;
            }
    }
    int x = robots.at(closestRobotIndex).getPos().x;
    int y = robots.at(closestRobotIndex).getPos().y;
    ROS_INFO("Updated robot: %i", robots.at(closestRobotIndex).getId());
    ROS_INFO("Updated robot pos: %i, %i", x, y);

    // eturn robotGagnant;
}
void subCallback(const elikos_ros::RobotRawArray::ConstPtr& msg) {
    // Init result image
    cv::Mat_<cv::Vec3b> img(400, 400, cv::Vec3b(0, 0, 255));

    ROS_INFO("In callback, num of robots =  %li", msg->robots.size());

    int indexRobotGagnant = NUM_ROBOTS_PER_COLOR;
    for (int i = 0; i < msg->robots.size(); i++) {
        if (msg->robots[i].color == RED) {
            indexRobotGagnant = DoMatch(msg->robots[i].point, redRobots);
            ROS_INFO("Found red robot: %i",
                     greenRobots.at(indexRobotGagnant).getId());
            greenRobots.at(indexRobotGagnant).setPos(msg->robots[i].point);
            //greenRobots.at(indexRobotGagnant).setFcu(msg->robots[i].poseFcu);

        } else if (msg->robots[i].color == GREEN) {
            indexRobotGagnant = DoMatch(msg->robots[i].point, greenRobots);
            ROS_INFO("Found green robot: %i",
                     greenRobots.at(indexRobotGagnant).getId());
            greenRobots.at(indexRobotGagnant).setPos(msg->robots[i].point);
            //greenRobots.at(indexRobotGagnant).setFcu(msg->robots[i].poseFcu);

        } else {
            ROS_ERROR("Color does not match, color is %i", msg->robots[i].color);
        }

        // Add result to image
        int x = msg->robots[i].point.x / 2;
        int y = msg->robots[i].point.y / 2;

        for (int i = x - 2; i < x + 2; i++) {
            for (int j = y - 2; j < y + 2; j++) {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
            }
        }
    }
    // Show image
    cv::imshow("Tracking-results", img);
    cv::waitKey(1);
}

void incertitudeCallback(const ros::TimerEvent& e) {
    ros::Duration diffTime = e.current_real - e.last_real;

    for (int id = 0; id < NUM_ROBOTS_PER_COLOR; id++) {
        redRobots.at(id).updateIncertitude(diffTime.nsec);
        greenRobots.at(id + NUM_ROBOTS_PER_COLOR)
            .updateIncertitude(diffTime.nsec);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_node");

    ros::NodeHandle n;
    cv::Mat_<cv::Vec3b> img(400, 400, cv::Vec3b(0, 0, 255));
    cv::namedWindow("Tracking-results", CV_WINDOW_AUTOSIZE);
    cv::imshow("Tracking-results", img);

    // Create all robots
    for (int id = 0; id < NUM_ROBOTS_PER_COLOR; id++) {
        // TODO: Decider si on fait un gros vecteur ou 2 petits pour chaque
        // couleur
        redRobots.push_back(Robot(id, RED));
        greenRobots.push_back(Robot(NUM_ROBOTS_PER_COLOR + id, GREEN));
    }

    ros::Subscriber sub =
        n.subscribe("/elikos_robot_raw_array", 1000, subCallback);

    // Timer pour calcul de l'incertitude
    // ros::Timer timer = n.createTimer(ros::Duration(0.1),
    // incertitudeCallback);
    ros::spin();

    cv::waitKey(1);

    return 0;
}
