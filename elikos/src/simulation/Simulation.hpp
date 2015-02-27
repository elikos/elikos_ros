#ifndef SIM_SIMULATION_HPP
#define SIM_SIMULATION_HPP

#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

#include "Robot.hpp"

namespace elikos_sim {

    class Simulation
    {
    public:
        Simulation(int& argc, char** argvm, ros::NodeHandle& node);
        ~Simulation();

        void Exec();

    private:
        bool checkCollision(const elikos_sim::Robot* ra, const elikos_sim::Robot* rb);
        double collisionAngle(tf::Vector3 v, double yaw);
        void setupArenaBoundaries(visualization_msgs::MarkerArray* arenaMarkers);
        bool static isOutOfBounds(const elikos_sim::Robot* robot);

        bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    private:
        double simSpeed, vel_xy_p, vel_xy_i, vel_xy_d, vel_z_p, vel_z_i, vel_z_d;
        double vel_xy_max, vel_z_max;
        int nTrgtRobots, nObsRobots, frameRate;
        ros::Rate r;
        visualization_msgs::Marker mavMarker;
        visualization_msgs::Marker setpointMarker;
        visualization_msgs::MarkerArray robotMarkers;
        ros::NodeHandle node;
        elikos_sim::MAV * mav;
        std::vector<elikos_sim::Robot*> robots;

        // Publishers
        ros::Publisher marker_pub;
        ros::Publisher mav_marker_pub;
        ros::Publisher setpoint_marker_pub;
        ros::Publisher arena_pub ;
        tf::TransformBroadcaster br;
        // Subscribers
        ros::Subscriber pose_sub;

        visualization_msgs::MarkerArray arenaMarkers;


        /* *************************************************************************************************
         * ***           HIDDEN CONSTRUCTORS (do not implement)
         * *************************************************************************************************
         */

        Simulation();
        Simulation& operator= (const Simulation&);
        Simulation (const Simulation&);
    };
}

#endif // SIM_SIMULATION_HPP
