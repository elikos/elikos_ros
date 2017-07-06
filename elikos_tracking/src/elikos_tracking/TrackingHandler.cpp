#include "TrackingHandler.h"

#define DEBUG 1

#define RED 0
#define GREEN 1

TrackingHandler::TrackingHandler() {

    //Init all robots
    for (int id = 0; id < NUM_ROBOTS_PER_COLOR; id++) {
        // TODO: Decider si on fait un gros vecteur ou 2 petits pour chaque
        // couleur
        redRobots.push_back(std::unique_ptr<Robot>(new Robot(id, RED)));
        greenRobots.push_back(std::unique_ptr<Robot>(
            new Robot(NUM_ROBOTS_PER_COLOR + id, GREEN)));
    }
}

TrackingHandler::~TrackingHandler()
{}

void TrackingHandler::init()
{
   TrackingHandler* handler = new TrackingHandler();
    trackHandler = handler;
}

int TrackingHandler::DoMatch(geometry_msgs::Point inputPoint, uint8_t color) {
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
    // std::vector<Robot*>* currRobotsVec;
    // if (color == RED) currRobotsVec = &redRobots;
    // else if (color == GREEN) currRobotsVec = &greenRobots;
    // else
    //{
    //    ROS_ERROR("Colors do not match!");
    //    return -1;
    // }
    int closestRobotIndex = redRobots.size();
    for (int i = 0; i < redRobots.size(); i++) {
        if (redRobots.at(i) == nullptr){
            ROS_ERROR("Nullptr");
            break;
        }
        if (!redRobots.at(i)->isNew) {

            double robotDist = redRobots.at(i)->getDistanceFrom(inputPoint);

            // On prend le robot le plus proche de la inputPose
            if (robotDist < dist) {
                // TODO: prendre en compte l'incertitude?
                dist = robotDist;
                closestRobotIndex = i;
            }
        }
    }
    ROS_INFO("end of for");
    /*
        if (closestRobotIndex) {
            // Si tous les robots sont nouveaux, on assigne au premier
       disponible
            for (int i = 0; i < currRobotsVec->size(); i++)
                if (currRobotsVec->at(i)->isNew) {
                    currRobotsVec->at(i)->isNew = false;
                    closestRobotIndex = i;
                    break;
                }
        }
        int x = currRobotsVec->at(closestRobotIndex)->getPos().x;
        int y = currRobotsVec->at(closestRobotIndex)->getPos().y;
        ROS_INFO("Updated robot: %i",
       currRobotsVec->at(closestRobotIndex)->getId());
        ROS_INFO("Updated robot pos: %i, %i", x, y);

        // eturn robotGagnant;
        */
}
void TrackingHandler::subCallback(
    const elikos_ros::RobotRawArray::ConstPtr& msg) {
    // Init result image
    cv::Mat_<cv::Vec3b> img(400, 400, cv::Vec3b(0, 0, 255));

    ROS_INFO("In callback, num of robots =  %li", msg->robots.size());

    int indexRobotGagnant = NUM_ROBOTS_PER_COLOR;
    int idGagnant = -1;
    for (int i = 0; i < msg->robots.size(); i++) {
        indexRobotGagnant = getHandler()->DoMatch(msg->robots[i].point, msg->robots[i].color);

        /* if (msg->robots[i].color == RED) {
             indexRobotGagnant = DoMatch(msg->robots[i].point, redRobots);
             ROS_INFO("Found red robot: %i",
                      redRobots.at(indexRobotGagnant).getId());
             redRobots.at(indexRobotGagnant).setPos(msg->robots[i].point);
             idGagnant = redRobots.at(indexRobotGagnant).getId();
             // redRobots.at(indexRobotGagnant).setFcu(msg->robots[i].poseFcu);

         } else if (msg->robots[i].color == GREEN) {
             indexRobotGagnant = DoMatch(msg->robots[i].point, greenRobots);
             ROS_INFO("Found green robot: %i",
                      greenRobots.at(indexRobotGagnant).getId());
             greenRobots.at(indexRobotGagnant).setPos(msg->robots[i].point);
             idGagnant = greenRobots.at(indexRobotGagnant).getId();
             //
         greenRobots.at(indexRobotGagnant).setFcu(msg->robots[i].poseFcu);

         } else {
             ROS_ERROR("Color does not match, color is %i",
                       msg->robots[i].color);
         }
         */
        // Add result to image
        int x = msg->robots[i].point.x / 2;
        int y = msg->robots[i].point.y / 2;
        // Init font
        // CvFont* font;
        //  cv::cvInitFont(font, cv::CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);

        for (int i = x - 2; i < x + 2; i++) {
            for (int j = y - 2; j < y + 2; j++) {
                // img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
                cv::putText(img, std::to_string(idGagnant), cv::Point(x, y),
                            CV_FONT_HERSHEY_SIMPLEX, 1.0,
                            cv::Scalar(0, 255, 0));
            }
        }
    }
    // Show image
    cv::imshow("Tracking-results", img);
    cv::waitKey(1);
}