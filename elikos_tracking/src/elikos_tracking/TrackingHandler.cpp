#include "TrackingHandler.h"

#define DEBUG 1

#define RED 2
#define GREEN 1

TrackingHandler* TrackingHandler::handlerInstance_ = 0;

TrackingHandler::TrackingHandler() {
    // Init all robots
    // TODO : assign initial position ?
    for (int id = 0; id < NUM_ROBOTS_PER_COLOR; id++) {
        robotsVec_.push_back(std::make_shared<Robot>(id, RED));
    }
    for (int id = NUM_ROBOTS_PER_COLOR; id < NUM_ROBOTS_PER_COLOR * 2; id++) {
        robotsVec_.push_back(std::make_shared<Robot>(id, GREEN));
    }
}

TrackingHandler* TrackingHandler::getInstance() {
    if (!handlerInstance_) {
        handlerInstance_ = new TrackingHandler();
    }
    return handlerInstance_;
}

int TrackingHandler::DoMatch(geometry_msgs::Point inputPoint, uint8_t color) {
    // Check color
    if (!(color == RED || color == GREEN)) {
        ROS_ERROR("Colors do not match!");
        return -1;
    }
    double dist = DBL_MAX;
    int closestRobotIndex = -1;
    for (int i = 0; i < robotsVec_.size(); i++) {
        if (robotsVec_.at(i) == nullptr) {
            ROS_ERROR("Nullptr");
            break;
        }
        if (!robotsVec_.at(i)->isNew && robotsVec_.at(i)->getColor() == color &&
            !robotsVec_.at(i)->getAssigned()) {
            double robotDist = robotsVec_.at(i)->getDistanceFrom(inputPoint);
            // On prend le robot le plus proche de la inputPose
            if (robotDist < dist) {
                // TODO: prendre en compte l'incertitude?
                dist = robotDist;
                closestRobotIndex = i;
            }
        }
    }

    if (closestRobotIndex == -1) {
        // Si tous les robots sont nouveaux, on assigne au premier disponible de
        // la meme couleur
        ROS_ERROR("All robots are new");
        for (int i = 0; i < robotsVec_.size(); i++)
            if (robotsVec_.at(i)->isNew &&
                robotsVec_.at(i)->getColor() == color &&
                !robotsVec_.at(i)->getAssigned()) {
                robotsVec_.at(i)->isNew = false;
                closestRobotIndex = i;
                break;
            }
    }
    if (closestRobotIndex == -1) {
        // Si on a toujours pas trouve de robot
        ROS_ERROR("No matching robot found.");
        return -1;
    }
    int x = robotsVec_.at(closestRobotIndex)->getPos().x;
    int y = robotsVec_.at(closestRobotIndex)->getPos().y;
    ROS_INFO("Updated robot: %i", robotsVec_.at(closestRobotIndex)->getId());
    ROS_INFO("Updated robot pos: %i, %i", x, y);

    return closestRobotIndex;
}

std::shared_ptr<Robot> TrackingHandler::getRobotAtIndex(int index) {
    return robotsVec_.at(index);
}

void TrackingHandler::subCallback(
    const elikos_ros::RobotRawArray::ConstPtr& msg) {
    ROS_INFO("In callback, num of robots =  %li", msg->robots.size());

    for (int i = 0; i < msg->robots.size(); i++) {
        ROS_INFO("here i = %i, color = %i", i, msg->robots[i].color);
        int indexRobotGagnant = NUM_ROBOTS_PER_COLOR;
        indexRobotGagnant =
            getInstance()->DoMatch(msg->robots[i].point, msg->robots[i].color);
        if (indexRobotGagnant != -1) {
            // Get robot
            std::shared_ptr<Robot> robotGagnant =
                getInstance()->getRobotAtIndex(indexRobotGagnant);
            robotGagnant->setAssigned(true);
            robotGagnant->setPos(msg->robots[i].point);
        }
    }
    getInstance()->clearRobots();
    getInstance()->drawResultImage();
}

void TrackingHandler::drawResultImage() {
    // Init result image
    cv::Mat_<cv::Vec3b> img(400, 400, cv::Vec3b(255, 255, 255));

    for (int i = 0; i < robotsVec_.size(); i++) {
        if (!robotsVec_.at(i)->isNew) {
            // Add result to image
            int x = robotsVec_.at(i)->getPos().x / 2;
            int y = robotsVec_.at(i)->getPos().y / 2;

            cv::Scalar textColor;
            if (robotsVec_.at(i)->getColor() == GREEN) {
                textColor = cv::Scalar(0, 255, 0);

            } else if (robotsVec_.at(i)->getColor() == RED) {
                textColor = cv::Scalar(0, 0, 255);
            } else {
                // Write in black
                textColor = cv::Scalar(255, 255, 255);
            }
            cv::putText(img, std::to_string(robotsVec_.at(i)->getId()), cv::Point(x, y),
                        CV_FONT_HERSHEY_SIMPLEX, 1.0, textColor, 2);
        }
    }
    // Show image
    cv::imshow("Tracking-results", img);
    cv::waitKey(1);
}
void TrackingHandler::clearRobots() {
    // Removed all assigned tags
    for (int i = 0; i < robotsVec_.size(); i++) {
        getInstance()->getRobotAtIndex(i)->setAssigned(false);
    }
}