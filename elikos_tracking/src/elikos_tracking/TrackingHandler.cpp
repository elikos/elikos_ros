#include "TrackingHandler.h"

#define DEBUG 1

#define RED 2
#define GREEN 1

TrackingHandler *TrackingHandler::handlerInstance_ = 0;

TrackingHandler::TrackingHandler()
{
    // Init all robots
    // TODO : assign initial position ?
    for (int id = 0; id < NUM_ROBOTS_PER_COLOR; id++)
    {
        robotsVec_.push_back(std::make_shared<Robot>(id, RED));
    }
    for (int id = NUM_ROBOTS_PER_COLOR; id < NUM_ROBOTS_PER_COLOR * 2; id++)
    {
        robotsVec_.push_back(std::make_shared<Robot>(id, GREEN));
    }

    //Init publisher
    ros::NodeHandle n;
    targetsPub_ = n.advertise<elikos_ros::TargetRobotArray>("/elikos_track_robot_array", 1000);
    debugPub_ = n.advertise<visualization_msgs::MarkerArray>("/elikos_track_debug", 1);

    marker_.header.frame_id = "elikos_arena_origin";

    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = 0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.2;
    marker_.scale.y = 0.2;
    marker_.scale.z = 0.2;
    marker_.color.a = 1.0; // Don't forget to set the alpha!
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
    marker_.lifetime.sec = 0;
}

TrackingHandler *TrackingHandler::getInstance()
{
    if (!handlerInstance_)
    {
        handlerInstance_ = new TrackingHandler();
    }
    return handlerInstance_;
}
/*
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
    float x = robotsVec_.at(closestRobotIndex)->getPos().x;
    float y = robotsVec_.at(closestRobotIndex)->getPos().y;
    ROS_INFO("Updated robot: %i", robotsVec_.at(closestRobotIndex)->getId());
    ROS_INFO("Updated robot pos: %f, %f", x, y);

    return closestRobotIndex;
}
*/
std::shared_ptr<Robot> TrackingHandler::getRobotAtIndex(int index)
{
    return robotsVec_.at(index);
}
void TrackingHandler::AssignRobots(
    const elikos_ros::TargetRobotArray::ConstPtr &msg)
{
    std::vector<double> ModelMsgDistancesForRedRobots(NUM_ROBOTS_PER_COLOR *
                                                      msg->targets.size());
    std::vector<double> ModelMsgDistancesForGreenRobots(NUM_ROBOTS_PER_COLOR *
                                                        msg->targets.size());

    for (int i = 0; i < msg->targets.size(); i++)
    {
        ROS_INFO("here i = %i, color = %i", i, msg->targets[i].color);
        // Creation de tableaux indiquant la distance de chaque robot du modele
        // a chaque robot du message
        for (int j = 0; j < NUM_ROBOTS_PER_COLOR; j++)
        {
            if (msg->targets[i].color == RED)
            {
                ModelMsgDistancesForRedRobots.at(i * NUM_ROBOTS_PER_COLOR + j) =
                    robotsVec_.at(j)->getDistanceFrom(
                        msg->targets[i].poseOrigin.pose.position);
            }
            else if (msg->targets[i].color == GREEN)
            {
                ModelMsgDistancesForGreenRobots.at(i * NUM_ROBOTS_PER_COLOR +
                                                   j) =
                    robotsVec_.at(NUM_ROBOTS_PER_COLOR + j)
                        ->getDistanceFrom(
                            msg->targets[i].poseOrigin.pose.position);
            }
            else
            {
                ROS_ERROR("Color is neither green nor red.");
            }
        }
    }

    // On prend le plus petit de chaque tableau, puis on elimine la colonne et
    // la ligne correspondante, jusqu'a temps que le tableau soit vide
    // Rouge
    for (int i = 0; i < msg->targets.size(); i++)
    {
        int idxMinDist = -1;
        double minDist = DBL_MAX;
        for (int j = 0; j < ModelMsgDistancesForRedRobots.size(); j++)
        {
            if ((ModelMsgDistancesForRedRobots.at(j) >= 0) &&
                (ModelMsgDistancesForRedRobots.at(j) < minDist))
            {
                idxMinDist = j;
                minDist = ModelMsgDistancesForRedRobots.at(j);
            }
        }
        if (idxMinDist != -1)
        {
            int idxRobotModel =
                idxMinDist %
                NUM_ROBOTS_PER_COLOR; // Reste de la division entiere
            int idxRobotMsg =
                (int)(idxMinDist / NUM_ROBOTS_PER_COLOR); // Division entiere
            // On update la position du robot correspondant dans le modele
            robotsVec_.at(idxRobotModel)
                ->setPos(msg->targets[idxRobotMsg].poseOrigin.pose.position);
            // On elimine la colonne et la ligne
            for (int ligne = 0; ligne < NUM_ROBOTS_PER_COLOR; ligne++)
            {
                for (int colonne = 0; colonne < msg->targets.size();
                     colonne++)
                {
                    if (ligne == idxRobotModel || colonne == idxRobotMsg)
                    {
                        ModelMsgDistancesForRedRobots.at(
                            ligne + colonne * NUM_ROBOTS_PER_COLOR) = -1;
                    }
                }
            }
        }
        else
        {
            ROS_ERROR("Did not match any robot for i = %i", i);
        }
    }

    // Vert
    for (int i = 0; i < msg->targets.size(); i++)
    {
        int idxMinDist = -1;
        double minDist = DBL_MAX;
        for (int j = 0; j < ModelMsgDistancesForGreenRobots.size(); j++)
        {
            if (ModelMsgDistancesForGreenRobots.at(j) > 0 &&
                ModelMsgDistancesForGreenRobots.at(j) < minDist)
            {
                idxMinDist = j;
                minDist = ModelMsgDistancesForGreenRobots.at(j);
            }
        }
        if (idxMinDist != -1)
        {
            int idxRobotModel =
                idxMinDist % NUM_ROBOTS_PER_COLOR; // Reste de la division entiere
            int idxRobotMsg =
                (int)(idxMinDist / NUM_ROBOTS_PER_COLOR); // Division entiere
            // On update la position du robot correspondant dans le modele
            robotsVec_.at(NUM_ROBOTS_PER_COLOR + idxRobotModel)
                ->setPos(msg->targets[idxRobotMsg].poseOrigin.pose.position);
            // On elimine la colonne et la ligne
            for (int ligne = 0; ligne < NUM_ROBOTS_PER_COLOR; ligne++)
            {
                for (int colonne = 0; colonne < msg->targets.size(); colonne++)
                {
                    if (ligne == idxRobotModel || colonne == idxRobotMsg)
                    {
                        ModelMsgDistancesForGreenRobots.at(
                            ligne + colonne * NUM_ROBOTS_PER_COLOR) = -1;
                    }
                }
            }
        }

        else
        {
            ROS_ERROR("Did not match any robot for i = %i", i);
        }
    }
}

void TrackingHandler::subCallback(
    const elikos_ros::TargetRobotArray::ConstPtr &msg)
{
    ROS_INFO("In callback, num of robots = %li", msg->targets.size());

    getInstance()->AssignRobots(msg);

    // getInstance()->clearRobots();
    getInstance()->drawResultImage();
    getInstance()->publishTargets();
}

void TrackingHandler::drawResultImage()
{
    // Init result image
    cv::Mat_<cv::Vec3b> img(400, 400, cv::Vec3b(255, 255, 255));

    for (int i = 0; i < robotsVec_.size(); i++)
    {
        // if (!robotsVec_.at(i)->isNew) {
        // Add result to image
        float x = robotsVec_.at(i)->getPos().x * 20 + 200;
        float y = robotsVec_.at(i)->getPos().y * 20 + 200;

        cv::Scalar textColor;
        if (robotsVec_.at(i)->getColor() == GREEN)
        {
            textColor = cv::Scalar(0, 255, 0);
        }
        else if (robotsVec_.at(i)->getColor() == RED)
        {
            textColor = cv::Scalar(0, 0, 255);
        }
        else
        {
            // Write in black
            textColor = cv::Scalar(255, 255, 255);
        }
        cv::putText(img, std::to_string(robotsVec_.at(i)->getId()),
                    cv::Point(x, y), CV_FONT_HERSHEY_SIMPLEX, 1.0, textColor,
                    2);
        // }
    }
    // Show image
    cv::imshow("Tracking-results", img);
    cv::waitKey(1);
}

void TrackingHandler::publishTargets()
{
    elikos_ros::TargetRobotArray msgTargetArray;
    //Debug
    visualization_msgs::MarkerArray msgDebugArray;
    for (int i = 0; i < robotsVec_.size(); i++)
    {
       elikos_ros::TargetRobot msg;
       msg.id = robotsVec_.at(i)->getId();
       msg.color = robotsVec_.at(i)->getColor();
       msg.poseOrigin.pose.position = robotsVec_.at(i)->getPos();
       msg.poseOrigin.header.stamp = ros::Time::now();
       msg.poseOrigin.header.frame_id = "elikos_arena_origin";

       msgTargetArray.targets.push_back(msg);

       //Debug publisher
       //marker_.pose.position = robotsVec_.at(i)->getPos();
       marker_.id = i;
        marker_.pose.position = robotsVec_.at(i)->getPos();
       marker_.header.stamp = ros::Time::now();
        marker_.color.r = 255 * (i % 2);
        marker_.color.g = 255 * ((i/2) % 2);
        marker_.color.b = 255 * ((i/4) % 2);

        if (i == robotsVec_.size()-1)
        {
            marker_.color.r = 0;
            marker_.color.g = 120;
            marker_.color.b = 255;
        }
        msgDebugArray.markers.push_back(marker_);
    }
    targetsPub_.publish(msgTargetArray);
    debugPub_.publish(msgDebugArray);
}
void TrackingHandler::clearRobots()
{
    // Removed all assigned tags
    for (int i = 0; i < robotsVec_.size(); i++)
    {
        getInstance()->getRobotAtIndex(i)->setAssigned(false);
    }
}