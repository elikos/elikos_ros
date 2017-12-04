#include <ros/ros.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

static const double LOOP_RATE_SIM = 10.0;

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

    ros::NodeHandle n_p("~");
    std::string poseTopic, tfName, tfBaseName;
    n_p.getParam("pose_topic", poseTopic);
    n_p.getParam("tf_name", tfName);
    n_p.getParam("base_tf_name", tfBaseName);

    // init values
    x = 0.0;
    y = 0.0;
    z = 0.0;
    yaw = tf::Quaternion(0.0, 0.0, 0.0, 1.0);

    // subscriber
    ros::Subscriber pose_sub = n.subscribe(poseTopic, 10, &poseCallback);

    while (ros::ok())
    {
        // publish setpoint tf
        tf::Transform tf;
        tf.setOrigin(tf::Vector3(x, y, z));
        tf.setRotation(yaw);
        tf_br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), tfBaseName, tfName));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}