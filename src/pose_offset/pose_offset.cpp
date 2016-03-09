#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include "PoseOffsetNode.h"


int main(int argc, char **argv) {
    PoseOffsetNode poseOffsetNode;

    poseOffsetNode.main(argc, argv);

    return 0;
}