//
// Created by andre on 22/07/15.
//

#ifndef ELIKOS_ROS_POSITIONHELPER_H
#define ELIKOS_ROS_POSITIONHELPER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

namespace elikos {
    namespace tools {
        static geometry_msgs::Point AddPoints(geometry_msgs::Point a, geometry_msgs::Point b) {
            geometry_msgs::Point p;
            p.x = a.x + b.x;
            p.y = a.y + b.y;
            p.z = a.z + b.z;
            return p;
        }

        static geometry_msgs::Quaternion
    }
}

#endif //ELIKOS_ROS_POSITIONHELPER_H
