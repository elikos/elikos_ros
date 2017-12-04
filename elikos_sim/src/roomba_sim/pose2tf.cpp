#include <ros/ros.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

static const double LOOP_RATE_SIM = 10.0;
// names
static const std::string QUAD_TF_NAME_BASE = "/elikos_arena_origin";                 // origin
static const std::string QUAD_TF_NAME_SETPOINT = "/elikos_setpoint";                 // quad setpoint tf

double x, y, z;
tf::Quaternion yaw;

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // update pose info
    x = msg->position.x;
    y = msg->position.y;
    z = msg->position.z;
    tf::quaternionMsgToTF(msg->orientation, yaw);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose2tf");
    ros::NodeHandle n;
    ros::Rate rate(LOOP_RATE_SIM);

    tf::TransformBroadcaster tf_br;

    //ros::NodeHandle n_p("~");
    //int nbTargetRobots, nbObstacleRobots;
    //n_p.getParam("/target_robot_count", nbTargetRobots);
    //n_p.getParam("/obstacle_robot_count", nbObstacleRobots);

    // init values
    x = 0.0;
    y = 0.0;
    z = 0.0;
    yaw = tf::Quaternion(0.0, 0.0, 0.0, 1.0);

    // subscriber
    ros::Subscriber pose_sub = n.subscribe("elikos_setpoint_pose", 10, &poseCallback);

    while (ros::ok())
    {
        // publish setpoint tf
        tf::Transform tf;
        tf.setOrigin(tf::Vector3(x, y, z));
        tf.setRotation(yaw);
        tf_br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), QUAD_TF_NAME_BASE, QUAD_TF_NAME_SETPOINT));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}