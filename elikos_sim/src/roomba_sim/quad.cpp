#include "quad.h"

Quad::Quad(ros::NodeHandle& n, ros::Duration expectedCycleTime)
    : n_(n),
      expectedCycleTime_(expectedCycleTime)
{
    // setup publishers
    quad_marker_pub_ = n.advertise<visualization_msgs::Marker>(QUAD_MARKER_TOPIC_NAME, 10);
    setpoint_marker_pub_ = n.advertise<visualization_msgs::Marker>(SETPOINT_MARKER_TOPIC_NAME, 10);

    // get params
    ros::NodeHandle n_p("~");
    double vel_xy_p, vel_xy_i, vel_xy_d, vel_xy_max, vel_z_p, vel_z_i, vel_z_d, vel_z_max;
    n_p.param<double>("/vel_xy_p", vel_xy_p, 2.0);
    n_p.param<double>("/vel_xy_i", vel_xy_i, 0.5);
    n_p.param<double>("/vel_xy_d", vel_xy_d, 1.0);
    n_p.param<double>("/vel_xy_max", vel_xy_max, 1.0);
    n_p.param<double>("/vel_z_p", vel_z_p, 2.0);
    n_p.param<double>("/vel_z_i", vel_z_i, 0.5);
    n_p.param<double>("/vel_z_d", vel_z_d, 1.0);
    n_p.param<double>("/vel_z_max", vel_z_max, 0.5);
    // tfs
    n_p.getParam("tf_origin", tfOrigin_);
    n_p.getParam("tf_pose", tfPose_);
    n_p.getParam("tf_setpoint", tfSetpoint_);

    // setup PIDs
    pid_vel_x_ = new BoundedPID(-vel_xy_max, vel_xy_max, vel_xy_p, vel_xy_i, vel_xy_d, 0.0, -0.0, false);
    pid_vel_y_ = new BoundedPID(-vel_xy_max, vel_xy_max, vel_xy_p, vel_xy_i, vel_xy_d, 0.0, -0.0, false);
    pid_vel_z_ = new BoundedPID(-vel_z_max, vel_z_max, vel_z_p, vel_z_i, vel_z_d, 0.0, -0.0, false);
    

    // wait for transforms
    tf_listener_.waitForTransform(tfOrigin_, tfSetpoint_, ros::Time(0), ros::Duration(1.0));
}

Quad::~Quad() {
    delete pid_vel_x_;
    delete pid_vel_y_;
    delete pid_vel_z_;
}

/*===========================
 * Other utilities
 *===========================*/

visualization_msgs::Marker Quad::createMarkerMsg(std::string frame, std::string meshResource, double r, double g, double b, double a) {
    visualization_msgs::Marker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame;
    msg.type = visualization_msgs::Marker::MESH_RESOURCE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.mesh_resource = QUAD_MESH_RESOURCE_PREFIX + meshResource;
    msg.pose.orientation.w = 1.0;
    msg.color.r = r;
    msg.color.g = g;
    msg.color.b = b;
    msg.color.a = a;
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.mesh_use_embedded_materials = true;
    msg.lifetime = ros::Duration();
    return msg;
}

double Quad::getInteractionDiameter() const {
    return INTERACTION_DIAMETER;
}

tf::Vector3 Quad::getPosition() const {
    return tf::Vector3(pos_x_, pos_y_, pos_z_);
}

/*===========================
 * Update
 *===========================*/

void Quad::publishQuad() {
    // publish elikos_fcu
    tf::Transform tf;
    tf::Quaternion q;
    tf.setOrigin(tf::Vector3(pos_x_, pos_y_, pos_z_));
    q.setRPY(0, 0, yaw_);
    tf.setRotation(q);
    tf_br_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), tfOrigin_, tfPose_));

    // publish elikos_fcu_marker
    quad_marker_pub_.publish(createMarkerMsg(tfPose_, QUAD_MARKER_MODEL_NAME, 0.8, 0.0, 0.0, 1.0));
}

void Quad::publishSetpointMarker() {
    // publish elikos_setpoint_marker
    setpoint_marker_pub_.publish(createMarkerMsg(tfSetpoint_, QUAD_MARKER_MODEL_NAME, 0.0, 0.8, 0.0, 0.25));
}

void Quad::updatePose() {
    pos_x_ += vel_x_ * expectedCycleTime_.toSec();
    pos_y_ += vel_y_ * expectedCycleTime_.toSec();
    pos_z_ += vel_z_ * expectedCycleTime_.toSec();
    //yaw_ += vel_yaw_ * expectedCycleTime_.toSec();
}

void Quad::updateVel() {
    vel_x_ = pid_vel_x_->computeCommand(setpoint_x_ - pos_x_, expectedCycleTime_);
    vel_y_ = pid_vel_y_->computeCommand(setpoint_y_ - pos_y_, expectedCycleTime_);
    vel_z_ = pid_vel_z_->computeCommand(setpoint_z_ - pos_z_, expectedCycleTime_);
    //vel_yaw_
}

void Quad::updateSetpoint() {
    try {
        tf_listener_.lookupTransform(tfOrigin_, tfSetpoint_, ros::Time(0), currentSetpoint_);
    } catch (tf::TransformException e) {
        ROS_ERROR("QUAD tf setpoint: %s", e.what());
    }
    setpoint_x_ = currentSetpoint_.getOrigin().x();
    setpoint_y_ = currentSetpoint_.getOrigin().y();
    setpoint_z_ = currentSetpoint_.getOrigin().z();
    //setpoint_yaw_ = tf::getYaw(currentSetpoint_.getRotation());
}

void Quad::update() {
    updateSetpoint();
    updateVel();
    updatePose();
    publishSetpointMarker();
    publishQuad();
}
 /*
void Quad::spinOnce() {
  update();
  ros::spinOnce();
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_quad");
    ros::NodeHandle n;

    ros::Rate rate(10.0);

    Quad quad_(n, rate.expectedCycleTime());
    
    try
    {
        while (ros::ok())
        {
            quad_.spinOnce();
            rate.sleep();
        }
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[SIM QUAD] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}
*/