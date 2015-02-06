#include <ros/ros.h>
#include <tf/tf.h>
#include "MAV.h"

namespace elikos_sim {

MAV::MAV(double simulationSpeed, ros::Duration cycleTime):
simSpeed(simulationSpeed), Name("MAV"), cycleTime(cycleTime) {
    x = 0;
    y = 0;
    z = 0;
    yaw = 0;
    vel_x_pid = new Pid<double>(0.0, 0.0, 0.0, // PID
                                Pid<double>::PID_DIRECT, // Controller direction
                                Pid<double>::ACCUMULATE_OUTPUT, // Output mode
                                33.3 / simSpeed, // Sample period
                                -5.0, 5.0, 0.0); // Min output, Max output, Setpoint
    vel_y_pid = new Pid<double>(0.0, 0.0, 0.0,
                                Pid<double>::PID_DIRECT,
                                Pid<double>::ACCUMULATE_OUTPUT,
                                33.3 / simSpeed,
                                -5.0, 5.0, 0.0);
    vel_z_pid = new Pid<double>(0.0, 0.0, 0.0,
                                Pid<double>::PID_DIRECT,
                                Pid<double>::ACCUMULATE_OUTPUT,
                                33.3 / simSpeed,
                                -5.0, 5.0, 0.0);
    refreshTransform();
};

MAV::~MAV(){
    delete vel_x_pid;
    delete vel_y_pid;
    delete vel_z_pid;
    vel_x_pid = NULL;
    vel_y_pid = NULL;
    vel_z_pid = NULL;
}

void MAV::setVelXYPID(double kp, double ki, double kd){
    vel_x_pid->SetTunings(kp, ki, kd);
    vel_x_pid->SetSamplePeriod((cycleTime.toSec() / simSpeed) * 1000);
    vel_y_pid->SetTunings(kp, ki, kd);
    vel_y_pid->SetSamplePeriod((cycleTime.toSec() / simSpeed) * 1000);
}

void MAV::setVelZPID(double kp, double ki, double kd){
    vel_z_pid->SetTunings(kp, ki, kd);
    vel_z_pid->SetSamplePeriod((cycleTime.toSec() / simSpeed) * 1000);
}

void MAV::setVelXYMax(double vel){
    vel_xy_max = vel;
    vel_x_pid->SetOutputLimits(-vel, vel);
    vel_y_pid->SetOutputLimits(-vel, vel);
}

void MAV::setVelZMax(double vel){
    vel_z_pid->SetOutputLimits(-vel, vel);
}

std::string MAV::getName(){
    return Name;
}

tf::Transform MAV::getTransform(){
    return t;
}

void MAV::move(){
    // Generate XY velocity setpoints
    vel_xy_sp.setX(x - xy_sp.getX());
    vel_xy_sp.setY(y - xy_sp.getY());
    if (vel_xy_sp.length() > vel_xy_max) {
        vel_xy_sp.normalize();
        vel_xy_sp *= vel_xy_max;
    }
    // Generate Z velocity setpoint
    vel_z_sp = z - z_sp;

    // Compute new XY velocities
    vel_x_pid->Run(vel_xy_sp.getX());
    vel_y_pid->Run(vel_xy_sp.getY());
    vel_xy.setX(vel_x_pid->output);
    vel_xy.setY(vel_y_pid->output);

    // Compute new Z velocity
    vel_z_pid->Run(vel_z_sp);
    vel_z = vel_z_pid->output;

    // Set new position
    x += vel_xy.getX() * cycleTime.toSec() * simSpeed;
    y += vel_xy.getY() * cycleTime.toSec() * simSpeed;
    z += vel_z * cycleTime.toSec() * simSpeed;
    // TODO: Yaw
    refreshTransform();
}

void MAV::collide(){
    // TODO
}

void MAV::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    xy_sp.setX(msg->pose.position.x);
    xy_sp.setY(msg->pose.position.y);
    z_sp = msg->pose.position.z;
    // TODO: Yaw
}

void MAV::refreshTransform(){
    v.setX(x);
    v.setY(y);
    v.setZ(z);
    q.setRPY(0, 0, yaw);
    t.setOrigin(v);
    t.setRotation(q);
}

visualization_msgs::Marker MAV::getVizMarker() {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = Name;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;
    marker.pose.position.x = t.getOrigin().getX();
    marker.pose.position.y = t.getOrigin().getY();
    marker.pose.position.z = t.getOrigin().getZ() + marker.scale.z / 2;
    marker.pose.orientation.x = t.getRotation().getX();
    marker.pose.orientation.y = t.getRotation().getY();
    marker.pose.orientation.z = t.getRotation().getZ();
    marker.pose.orientation.w = t.getRotation().getW();
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    return marker;
}

visualization_msgs::Marker MAV::getSetpointMarker() {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = Name;
    marker.id = 2;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;
    marker.pose.position.x = xy_sp.getX();
    marker.pose.position.y = xy_sp.getY();
    marker.pose.position.z = z_sp + marker.scale.z / 2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;
    marker.color.a = 0.5;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    return marker;
}

} // namespace elikos_sim
