#include "sim_detection.h"

SimDetection::SimDetection(ros::NodeHandle& n)
    : n_(n)
{
    // get params
    ros::NodeHandle n_p("~");
    n_p.getParam("/target_robot_count", nbTargetRobots_);
    //n_p.getParam("/obstacle_robot_count", nbObstacleRobots_);
    n_p.getParam("tf_origin", tfOrigin_);
    n_p.getParam("tf_pose", tfPose_);
    n_p.getParam("target_robots_topic", targetRobotArrayTopic_);
    n_p.getParam("target_robots_markers_topic", targetRobotArrayMarkerTopic_);
    n_p.getParam("detection_camera_info", detectionCameraInfo_str_);

    // convert string to vector
    detectionCameraInfo_ = getVectorFromString(detectionCameraInfo_str_);
    // throw error if size is inconsistent (not multiple of 3)
    if (detectionCameraInfo_.size() % 3 != 0) {
        ROS_ERROR("[SIM DETECTION] size of detection_camera_info array inconsistent. Using default.");
        detectionCameraInfo_ = {0.0, 1.5708, 20.0};
    }

    robotsPoses_ = new std::vector<tf::StampedTransform>(nbTargetRobots_);
    detectedRobots_ = new std::vector<elikos_main::TargetRobot>();

    // setup publishers
    testmarkers_pub_ = n_.advertise<geometry_msgs::PoseArray>(targetRobotArrayMarkerTopic_, 10);
    targetRobots_pub_ = n_.advertise<elikos_main::TargetRobotArray>(targetRobotArrayTopic_, 10);

    // wait for transforms
    tf_listener_.waitForTransform(tfOrigin_, tfPose_, ros::Time(0), ros::Duration(1.0));
    tf_listener_.waitForTransform(tfOrigin_, "/groundrobot1", ros::Time(0), ros::Duration(1.0));
}

SimDetection::~SimDetection() {
    delete robotsPoses_;
    delete detectedRobots_;
}

/*===========================
 * Other utilities
 *===========================*/

std::vector<double> SimDetection::getVectorFromString(std::string& str) {
    const std::string delim = ",";

    // remove square brackets []
    str = str.substr(1, str.size() - 2);

	std::vector<double> tokens;
	size_t prev = 0, pos = 0;
	do
	{
		pos = str.find(",", prev);
		if (pos == std::string::npos) pos = str.length();
		std::string token = str.substr(prev, pos - prev);
		if (!token.empty()) tokens.push_back(std::stod(token));
		prev = pos + 1;
	} while (pos < str.length() && prev < str.length());
	return tokens;
}

geometry_msgs::PoseStamped SimDetection::createPoseStampedFromPosYaw(tf::Vector3 pos, double yaw) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = pos.x();
    pose_msg.pose.position.y = pos.y();
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = tfOrigin_;
    return pose_msg;
}

double SimDetection::normalizeZeroTwoPi(double a) {
    while ((a > 2.0*PI) || (a < 0.0)) {
        if (a > 2.0*PI) {
            a -= 2.0*PI;
        } else if (a < 0.0) {
            a += 2.0*PI;
        }
    }
    return a;
}

bool SimDetection::isAngleWithin(double angle, double min, double max) {
    bool res;
    min = normalizeZeroTwoPi(min);
    max = normalizeZeroTwoPi(max);
    if (min > max) {
        // max overflowed (e.g. min=2.5, max=0.5)
        res = (angle >= min) || (angle <= max);
    } else {
        res = (angle >= min) && (angle <= max);
    }
    return res;
}

bool SimDetection::isDetected(double robotAngle, double robotDistanceSquared) {
    bool isDetected = false;

    robotAngle = normalizeZeroTwoPi(robotAngle);

    // for every camera
    for (int i = 0; i < (detectionCameraInfo_.size()/3); ++i) {
        // extract info
        double position = detectionCameraInfo_[(i*3) + 0];
        double angle = detectionCameraInfo_[(i*3) + 1];
        double distance = detectionCameraInfo_[(i*3) + 2];

        // calculate corresponding angle interval
        double angleMin = position - (angle / 2.0);
        double angleMax = position + (angle / 2.0);

        // check if it's within the current camera's FOV
        if (isAngleWithin(robotAngle, angleMin, angleMax)) {
            // check if it's within the current camera's range
            if (robotDistanceSquared <= pow(distance, 2)) {
                // detected
                isDetected = true;
            }
        }
    }

    return isDetected;
}

/*===========================
 * Update
 *===========================*/

void SimDetection::publishTestMarkers() {
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = tfOrigin_;

    std::vector<geometry_msgs::Pose> poses;
    for (auto& robot : *detectedRobots_) {
        poses.push_back(robot.poseOrigin.pose);
    }

    msg.poses = poses;

    testmarkers_pub_.publish(msg);
}

void SimDetection::publishTargetRobotArray() {
    elikos_main::TargetRobotArray msg;
    msg.targets = *detectedRobots_;
    targetRobots_pub_.publish(msg);
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
        double robot_x = robotsPoses_->at(i).getOrigin().x();
        double robot_y = robotsPoses_->at(i).getOrigin().y();

        double angleWrtQuad = atan2(robot_y - quad_y, robot_x - quad_x);
        double distanceSquaredWrtQuad = pow((robot_y - quad_y), 2) + pow((robot_x - quad_x), 2);

        if (isDetected(angleWrtQuad, distanceSquaredWrtQuad)) {
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
            tf_listener_.lookupTransform(tfOrigin_, "/groundrobot"+std::to_string(i+1), ros::Time(0), robotsPoses_->at(i));
        }
    } catch (tf::TransformException e) {
        ROS_ERROR("SIM DETECTION tf robots: %s", e.what());
    }
}

void SimDetection::updateQuadPose() {
    try {
        tf_listener_.lookupTransform(tfOrigin_, tfPose_, ros::Time(0), currentQuadPose_);
    } catch (tf::TransformException e) {
        ROS_ERROR("SIM DETECTION tf quad: %s", e.what());
    }
}

void SimDetection::update() {
    updateQuadPose();
    updateRobotsPoses();
    updateDetectedRobots();

    publishTargetRobotArray();
    publishTestMarkers();
}

void SimDetection::spinOnce()
{
  update();
  ros::spinOnce();
}

void SimDetection::spin()
{
  ros::Rate rate(10.0);
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