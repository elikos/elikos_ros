#ifndef ELIKOS_SIM_DETECTION_H
#define ELIKOS_SIM_DETECTION_H

/**
 * \file SimDetection.h
 * \brief SimDetection class declaration
 * \author christophebedard
 */

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "elikos_main/TargetRobotArray.h"

static const double PI = 3.1415;

/** \class SimDetection
 * \brief class which simulates target robot detection.
 *
 * It acquires the positions of all target robots, then filters which robots
 * would be detected given the info about its cameras, and publishes a 
 * elikos_main::TargetRobotArray message.
 */
class SimDetection
{
    public:
        /**
         * \brief SimDetection constructor.
         *
         * \param n : node handle.
         */
        SimDetection(ros::NodeHandle& n);

        /**
         * \brief Quad destructor.
         */
        ~SimDetection();

        /**
         * \brief ROS spin. Called only once (by node); contains ROS while loop.
         */
        void spin();

    protected:
        ros::NodeHandle& n_; /**< node handle */

        /*===========================
         * Update
         *===========================*/
        /**
         * \brief ROS spin once, called once every loop.
         */
        void spinOnce();

        /**
         * \brief Update; called every spinOnce().
         */
        void update();

        /**
         * \brief Update quad pose.
         */
        void updateQuadPose();

        /**
         * \brief Update target robots poses.
         */
        void updateRobotsPoses();

        /**
         * \brief Update detected robots.
         */
        void updateDetectedRobots();

        /**
         * \brief Publish test markers message.
         */
        void publishTestMarkers();

        /**
         * \brief Publish target robot array (main output of detection).
         */
        void publishTargetRobotArray();

    
    private:
        int nbTargetRobots_; /**< number of target robots */
        //int nbObstacleRobots_; /**< number of obstacle robots; in case we add obstacle robot detection */

        tf::StampedTransform currentQuadPose_; /**< quad pose */
        std::vector<tf::StampedTransform>* robotsPoses_; /**< targets poses */
        std::vector<elikos_main::TargetRobot>* detectedRobots_; /**< detected robots */

        std::string tfOrigin_; /**< arena origin tf */
        std::string tfPose_; /**< pose tf */
        std::string targetRobotArrayTopic_; /**< target detection array (output of detection) */
        std::string targetRobotArrayMarkerTopic_; /**< target detection pose array (for testing and viz) */

        std::vector<double> detectionCameraInfo_; /**< camera information vector (position (rad), angle (rad), range (m)) */
        std::string detectionCameraInfo_str_; /**< camera information in raw string format */

        tf::TransformListener tf_listener_; /**< tf listener used for robot pose and quad pose */

        /*===========================
         * Publishers
         *===========================*/
        ros::Publisher targetRobots_pub_; /**< detected target robots publisher (output of detection) */
        ros::Publisher testmarkers_pub_; /**< test markers publisher */

        /*===========================
         * Utilities
         *===========================*/
        /**
         * \brief Create PoseStamped message from position and yaw.
         *
         * \param pos : XYZ position.
         * \param yaw : heading.
         *
         * \return PoseStamped message.
         */
        geometry_msgs::PoseStamped createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw);

        /**
         * \brief Convert string "[0.0, 0.0, 0,0]" to vector<double>.
         *
         * \param str : raw string.
         *
         * \return vector.
         */
        std::vector<double> getVectorFromString(std::string& str);

        /**
         * \brief Check if robot is detected given its angle and distance^2 wrt quad.
         *
         * \param robotAngle : angle of robot wrt quad.
         * \param robotDistanceSquared : square of distance of robot wrt quad.
         *
         * \return result.
         */
        bool isDetected(double robotAngle, double robotDistanceSquared);

        /**
         * \brief Convert angle to [0,2pi] interval.
         *
         * \param a : angle.
         *
         * \return normalized angle.
         */
        double normalizeZeroTwoPi(double a);

        /**
         * \brief Check if angle is within [min,max] while handling cases where min>max.
         *
         * \param angle : angle.
         * \param min : minimum accepted value.
         * \param max : maximum accepted value.
         *
         * \return result.
         */
        bool isAngleWithin(double angle, double min, double max);
};

#endif  // ELIKOS_SIM_DETECTION_H