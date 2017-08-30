//
// Created by andre on 22/07/15.
//

#ifndef elikos_main_POSITIONHELPER_H
#define elikos_main_POSITIONHELPER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

namespace elikos {
    namespace tools {
        geometry_msgs::Point addPoints(geometry_msgs::Point a, geometry_msgs::Point b){
            geometry_msgs::Point p;
            p.x = a.x + b.x;
            p.y = a.y + b.y;
            p.z = a.z + b.z;
            return p;
        }

        geometry_msgs::Quaternion addQuaternions(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b){
            tf::Quaternion quat_tf_a, quat_tf_b;
            tf::quaternionMsgToTF(a, quat_tf_a);
            tf::quaternionMsgToTF(b, quat_tf_b);
            quat_tf_a += quat_tf_b;
            geometry_msgs::Quaternion ret;
            tf::quaternionTFToMsg(quat_tf_a, ret);
            return ret;
        }
    }
}

#endif //elikos_main_POSITIONHELPER_H
