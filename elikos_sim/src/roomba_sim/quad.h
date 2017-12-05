#ifndef ELIKOS_SIM_QUAD_H
#define ELIKOS_SIM_QUAD_H

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

class Quad
{
    public:
        Quad(ros::NodeHandle& n, ros::Duration expectedCycleTime);
        ~Quad();

        void update();
        void spinOnce();

        double getInteractionDiameter() const;
        tf::Vector3 getPosition() const;

    protected:
        void updateSetpoint();
        void updateVel();
        void updatePose();
        void publishSetpointMarker();
        void publishQuad();
        
    private:
        ros::NodeHandle& n_;

        /* arena origin tf */
        std::string tfOrigin_;
        /* pose tf */
        std::string tfPose_;
        /* setpoint tf */
        std::string tfSetpoint_;

        /* tf listener */
        tf::TransformListener tf_listener_;
        /* elikos_fcu tf publisher */
        tf::TransformBroadcaster tf_br_;
        /* Expected cycle time (1/refresh rate) */
        ros::Duration expectedCycleTime_;

        // publishers
        ros::Publisher quad_marker_pub_;
        ros::Publisher setpoint_marker_pub_;

        // setpoint
        tf::StampedTransform currentSetpoint_;
        double setpoint_x_, setpoint_y_, setpoint_z_;
        //double setpoint_yaw_;

        // pose
        double pos_x_, pos_y_, pos_z_;
        double yaw_;

        // velocities
        double vel_x_, vel_y_, vel_z_;
        //double vel_yaw_;

        // PIDs
        BoundedPID* pid_vel_x_;
        BoundedPID* pid_vel_y_;
        BoundedPID* pid_vel_z_;

        // Gains

        visualization_msgs::Marker createMarkerMsg(std::string frame, std::string meshResource, double r, double g, double b, double a);

};

#endif // ELIKOS_SIM_QUAD_H
