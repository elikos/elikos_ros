/**
* @brief    Ground robot behavior simulation for the IARC mission 7.
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <elikos_lib/GroundRobot.h>
#include <vector>

bool checkCollision(GroundRobot* robotA, GroundRobot* robotB);

double collisionAngle(tf::Vector3 v, double yaw);

void setVector(tf::Vector3 &v, double x, double y, double z);

visualization_msgs::Marker getRobotMarker(GroundRobot* robot);

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
    std::vector <GroundRobot *> robots;
    robots.reserve(totalRobots);
    visualization_msgs::MarkerArray robotMarkers;
    for (int i = 0; i < nTrgtRobots; i++) {
        robots.push_back(new GroundRobot(TARGET_ROBOT, nTrgtRobots, i, simSpeed));
        robotMarkers.markers.push_back(getRobotMarker(robots[i]));
    }
    for (int i = 0; i < nObsRobots; i++) {
        robots.push_back(new GroundRobot(OBSTACLE_ROBOT_RND, nObsRobots, i, simSpeed));
        robotMarkers.markers.push_back(getRobotMarker(robots[i]));
    }

    // Publishers
    ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("robotsim/robot_markers", 0);
    tf::TransformBroadcaster br;

    // Loop rate (Hz)
    ros::Rate r(frameRate);

    // Main Loop
    while (ros::ok()) {
        // Collision checking
        for (int i = 0; i < (totalRobots); i++) {
            for (int j = 0; j < totalRobots; j++) {
                if (i == j) continue;
                if (checkCollision(robots[i], robots[j])) {
                    robots[i]->collide();
                }
            }
        }
        // Advance robots to next frame and publish tf & marker
        for (int i = 0; i < totalRobots; i++) {
            robots[i]->advance(r.expectedCycleTime());
            robotMarkers.markers[i] = getRobotMarker(robots[i]);
            br.sendTransform(tf::StampedTransform(robots[i]->getTransform(), ros::Time::now(), "world", robots[i]->getName()));
        }
        marker_pub.publish(robotMarkers);
        r.sleep();
    }
    return 0;
};

bool checkCollision(GroundRobot* robotA, GroundRobot* robotB) {
    tf::Vector3 vA = robotA->getTransform().getOrigin();
    tf::Vector3 vB = robotB->getTransform().getOrigin();
    tf::Vector3 xAxis;
    setVector(xAxis, 1, 0, 0);
    tf::Quaternion qA = robotA->getTransform().getRotation();
//	if (robotA.getName() == "robot0" && robotB.getName() == "robot1")
//		std::cout << "Collision vector angle: " << atan2((vB-vA).getY(), (vB-vA).getX()) << "\tYaw:" << tf::getYaw(qA) << "\tcollisionAngle:" << collisionAngle(vB-vA, tf::getYaw(qA)) << "\n";

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

visualization_msgs::Marker getRobotMarker(GroundRobot* robot) {
    visualization_msgs::Marker marker;
    tf::Transform t = robot->getTransform();
    int r = 0, g = 0, b = 0;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = robot->getType();
    marker.id = robot->getID();
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = robot->getTypeID() ? 1.5 : 0.1;
    marker.pose.position.x = t.getOrigin().getX();
    marker.pose.position.y = t.getOrigin().getY();
    marker.pose.position.z = t.getOrigin().getZ() + marker.scale.z / 2;
    marker.pose.orientation.x = t.getRotation().getX();
    marker.pose.orientation.y = t.getRotation().getY();
    marker.pose.orientation.z = t.getRotation().getZ();
    marker.pose.orientation.w = t.getRotation().getW();

    switch (robot->getColor()) {
        case RED:
            r = 1.0;
            break;
        case GREEN:
            g = 1.0;
            break;
        case BLUE:
            b = 1.0;
            break;
        default :
            r = 1.0;
            g = 1.0;
            b = 1.0;
            break;
    }
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
}
