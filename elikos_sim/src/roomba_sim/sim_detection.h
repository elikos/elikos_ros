#ifndef ELIKOS_SIM_DETECTION_H
#define ELIKOS_SIM_DETECTION_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "elikos_main/TargetRobotArray.h"

// names
static const std::string TEST_MARKERS_TOPIC_NAME = "target_robot_array_test";   // display detected target in rviz

static const double PI = 3.1415;


class SimDetection
{
    public:
        /*
         * Constructor
         */
        SimDetection(ros::NodeHandle& n);
        ~SimDetection();

        /*
         * ROS spin. Called only once (by node); contains ROS while loop
         */
        void spin();

    protected:
        ros::NodeHandle& n_;

        /*===========================
         * Update
         *===========================*/
        /*
         * ROS spin once, called once every loop
         */
        void spinOnce();

        /*
         * Update; called every spinOnce()
         */
        void update();

        /*
         * Update quad pose
         */
        void updateQuadPose();

        /*
         * Update robot poses
         */
        void updateRobotsPoses();

        /*
         * Update detected robots
         */
        void updateDetectedRobots();

        /*
         * Publish test markers message
         */
        void publishTestMarkers();

        /*
         * Publish target robot array (main output of detection)
         */
        void publishTargetRobotArray();

    
    private:
        /* number of target robots */
        int nbTargetRobots_;
        /* number of obstacle robots; in case we add obstacle robot detection */
        //int nbObstacleRobots_;

        /* quad pose */
        tf::StampedTransform currentQuadPose_;
        /* targets poses */
        std::vector<tf::StampedTransform>* robotsPoses_;
        /* detected robots */
        std::vector<elikos_main::TargetRobot>* detectedRobots_;

        /* arena origin tf */
        std::string tfOrigin_;
        /* pose tf */
        std::string tfPose_;
        /* target detection array (output of detection) */
        std::string targetRobotArrayTopic_;
        /* target detection pose array (tests/viz) */
        std::string targetRobotArrayMarkerTopic_;

        /* camera information (position (rad), angle (rad), range (m)) */
        std::vector<double> detectionCameraInfo_;
        std::string detectionCameraInfo_str_;

        tf::TransformListener tf_listener_;

        /*===========================
         * Publishers
         *===========================*/
        /* Detected target robots publisher (output of detection) */
        ros::Publisher targetRobots_pub_;
        /* Test markers publisher */
        ros::Publisher testmarkers_pub_;

        /*===========================
         * Utilities
         *===========================*/
        /*
         * Create posestamped message from position and yaw
         */
        geometry_msgs::PoseStamped createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw);

        /*
         * Convert string "[0.0, 0.0, 0,0]" to vector<double>
         */
        std::vector<double> getVectorFromString(std::string& str);

        /*
         * Check if robot is detected given its angle and distance^2 wrt quad
         */
        bool isDetected(double robotAngle, double robotDistanceSquared);

        /*
         * Convert to [0,2pi]
         */
        double normalizeZeroTwoPi(double a);

        /*
         * Check if angle is within [min,max] while handling cases where min>max
         */
        bool isAngleWithin(double angle, double min, double max);
};

#endif  // ELIKOS_SIM_DETECTION_H