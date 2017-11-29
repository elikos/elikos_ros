#include "sim_detection.h"

SimDetection::SimDetection(ros::NodeHandle& n)
    : n_(n)
{
    // setup publishers
    testmarkers_pub_ = n.advertise<geometry_msgs::PoseArray>(TEST_MARKERS_TOPIC_NAME, 10);

    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("/target_robot_count", nbTargetRobots_);
    //n_p.getParam("/obstacle_robot_count", nbObstacleRobots_);

    angleMin_ = - 3.1415 / 4.0;
    angleMax_ = 3.1415 / 4.0;
    distanceMax_ = 20.0;

    robotsPoses_ = new std::vector<tf::StampedTransform>(nbTargetRobots_);
    detectedRobots_ = new std::vector<elikos_main::TargetRobot>();
}

SimDetection::~SimDetection() {
}

/*===========================
 * Other utilities
 *===========================*/

geometry_msgs::PoseStamped SimDetection::createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = TF_NAME_BASE;
    return pose_msg;
}

/*===========================
 * Update
 *===========================*/

void SimDetection::publishTestMarkers() {
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = TF_NAME_BASE;

    std::vector<geometry_msgs::Pose> poses;
    for (auto& robot : *detectedRobots_) {
        poses.push_back(robot.poseOrigin.pose);
    }

    msg.poses = poses;

    testmarkers_pub_.publish(msg);
}

void SimDetection::updateDetectedRobots() {
    // extract position and yaw
    double quad_x = currentQuadPose_.getOrigin().x();
    double quad_y = currentQuadPose_.getOrigin().y();
    double quad_z = currentQuadPose_.getOrigin().z();
    double quad_yaw = tf::getYaw(currentQuadPose_.getRotation());

    // clear detected robots
    detectedRobots_->clear();

    // loop through all robots and check if valid
    for (int i = 0; i < nbTargetRobots_; ++i) {
        //robotsPoses_->at(i)
        double robot_x = robotsPoses_->at(i).getOrigin().x();
        double robot_y = robotsPoses_->at(i).getOrigin().y();

        double angleWrtQuad = atan2(robot_y - quad_y, robot_x - quad_x);
        double distanceSquaredWrtQuad = pow((robot_y - quad_y), 2) + pow((robot_x - quad_x), 2);

        bool isDistanceOkay = distanceSquaredWrtQuad <= pow(distanceMax_, 2);
        bool isAngleOkay = (angleWrtQuad >= angleMin_) && (angleWrtQuad <= angleMax_);

        bool isDetected = isDistanceOkay && isAngleOkay;
        if (isDetected) {
            // create and add TargetRobot message
            elikos_main::TargetRobot msg;
            //msg.id = 0;
            //msg.color = 1;
            msg.poseOrigin = createPoseStampedFromPosYaw(tf::Vector3(robot_x, robot_y, 0.0), 0.0);
            detectedRobots_->push_back(msg);
        }
    }
}

void SimDetection::updateRobotsPoses() {
    try {
        for (int i = 0; i < nbTargetRobots_; ++i) {
            tf_listener_.lookupTransform(TF_NAME_BASE, "/groundrobot"+std::to_string(i+1), ros::Time(0), robotsPoses_->at(i));
        }
    } catch (tf::TransformException e) {
        ROS_ERROR("SIM DETECTION tf robots: %s", e.what());
    }
}

void SimDetection::updateQuadPose() {
    try {
        tf_listener_.lookupTransform(TF_NAME_BASE, TF_NAME_QUAD, ros::Time(0), currentQuadPose_);
    } catch (tf::TransformException e) {
        ROS_ERROR("SIM DETECTION tf quad: %s", e.what());
    }
}

void SimDetection::update() {
    updateQuadPose();
    updateRobotsPoses();
    updateDetectedRobots();
    publishTestMarkers();
}

void SimDetection::spinOnce()
{
  update();
  ros::spinOnce();
}

void SimDetection::spin()
{
  ros::Rate rate(30.0);
  while (ros::ok())
  {
    spinOnce();
    rate.sleep();
  }
}

// ---------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_detection");
    ros::NodeHandle n;

    SimDetection detect_(n);
    
    try
    {
        detect_.spin();
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL_STREAM("[SIM DETECTION] Runtime error: " << e.what());
        return 1;
    }
    return 0;
}