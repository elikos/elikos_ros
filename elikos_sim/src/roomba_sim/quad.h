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
static const std::string QUAD_TF_NAME_BASE = "/elikos_arena_origin";                 // origin
static const std::string QUAD_TF_NAME_SETPOINT = "/elikos_setpoint";                 // quad setpoint tf
static const std::string QUAD_TF_NAME = "/elikos_fcu";                          // quad pose tf
static const std::string QUAD_MESH_RESOURCE_PREFIX = "package://elikos_sim/src/gazebo_sim/elikos_gazebo_models/models/quad/"; //"package://elikos_gazebo_models/models/quad/"
static const std::string SETPOINT_MARKER_TOPIC_NAME = "elikos_setpoint_marker";
static const std::string QUAD_MARKER_TOPIC_NAME = "elikos_fcu_marker";
static const std::string QUAD_MARKER_MODEL_NAME = "Yopokos_vSim.dae";

class Quad
{
    public:
        Quad(ros::NodeHandle& n, ros::Duration expectedCycleTime);
        ~Quad();

        void update();
        void spinOnce();

    protected:
        void updateSetpoint();
        void updateVel();
        void updatePose();
        void publishSetpointMarker();
        void publishQuad();
        
    private:
        ros::NodeHandle& n_;

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
