#ifndef ELIKOS_SIM_QUAD_H
#define ELIKOS_SIM_QUAD_H

/**
 * \file quad.h
 * \brief Quad class declaration
 * \author christophebedard
 */

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "BoundedPID.h"

// names
static const std::string SETPOINT_MARKER_TOPIC_NAME = "markers/elikos_setpoint";
static const std::string QUAD_MARKER_TOPIC_NAME = "markers/elikos_fcu";
static const std::string QUAD_MESH_RESOURCE_PREFIX = "package://elikos_sim/src/gazebo_sim/elikos_gazebo_models/models/quad/"; //"package://elikos_gazebo_models/models/quad/"
static const std::string QUAD_MARKER_MODEL_NAME = "Yopokos_vSim.dae";

static const double INTERACTION_DIAMETER = 0.4;

/** \class Quad
 * \brief class which simulates the quadcopter.
 *
 * It moves to a given setpoint with simple PIDs and publishes a tf of its current pose.
 */
class Quad
{
    public:
        /**
         * \brief Quad constructor.
         *
         * \param n : node handle.
         * \param expectedCycleTime : expected time between every update.
         */
        Quad(ros::NodeHandle& n, ros::Duration expectedCycleTime);

        /**
         * \brief Quad destructor.
         */
        ~Quad();

        /**
         * \brief General update method, called every loop.
         */
        void update();

        /**
         * \brief Accessor for the diameter to consider for physical interactions.
         *
         * \return diameter.
         */
        double getInteractionDiameter() const;

        /**
         * \brief Accessor for the quad's current position.
         *
         * \return current position.
         */
        tf::Vector3 getPosition() const;

    private:
        ros::NodeHandle& n_; /**< node handle */

        std::string tfOrigin_; /**< arena origin tf */
        std::string tfPose_; /**< pose tf */
        std::string tfSetpoint_; /**< setpoint tf */

        tf::TransformListener tf_listener_; /**< tf listener used for getting setpoint */
        tf::TransformBroadcaster tf_br_; /**< tf broadcaster used for current quad pose */
        ros::Duration expectedCycleTime_; /**< expected cycle time (1/refresh rate) */

        ros::Publisher quad_marker_pub_; /**< quad marker publisher */
        ros::Publisher setpoint_marker_pub_; /**< setpoint marker publisher */

        tf::StampedTransform currentSetpoint_; /**< current setpoint tf */
        double setpoint_x_; /**< current X setpoint */
        double setpoint_y_; /**< current Y setpoint */
        double setpoint_z_; /**< current Z setpoint */
        //double setpoint_yaw_; /**< current yaw setpoint */

        double pos_x_; /**< current X position */
        double pos_y_; /**< current Y position */
        double pos_z_; /**< current Z position */
        double yaw_; /**< current yaw */

        double vel_x_; /**< X velocity */
        double vel_y_; /**< Y velocity */
        double vel_z_; /**< Z velocity */
        //double vel_yaw_; /**< yaw velocity */

        BoundedPID* pid_vel_x_; /**< X PID */
        BoundedPID* pid_vel_y_; /**< Y PID */
        BoundedPID* pid_vel_z_; /**< Z PID */

        /**
         * \brief Update setpoint by looking up latest setpoint tf.
         */
        void updateSetpoint();

        /**
         * \brief Update velocities by calling PIDs.
         */
        void updateVel();

        /**
         * \brief Update quad position given velocities.
         */
        void updatePose();

        /**
         * \brief Publish marker for current setpoint.
         */
        void publishSetpointMarker();

        /**
         * \brief Publish tf and marker for current quad position.
         */
        void publishQuad();

        /**
         * \brief Helper method for creating marker message.
         *
         * \param frame : frame_id to use.
         * \param meshResource : resource file to use.
         * \param r : red color value.
         * \param g : green color value.
         * \param b : blue color value.
         * \param a : transparency value.
         * 
         * \return marker message.
         */
        visualization_msgs::Marker createMarkerMsg(std::string frame, std::string meshResource, double r, double g, double b, double a);
};

#endif // ELIKOS_SIM_QUAD_H
