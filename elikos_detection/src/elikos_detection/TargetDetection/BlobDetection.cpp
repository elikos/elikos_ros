#include "BlobDetection.h"
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

int MAX_CIRCLE_RADIUS = 20;
int MIN_CIRCLE_RADIUS = 1;

/* PRE PROCESSING TEST*/
tf::TransformListener* _tfListener = nullptr;
void preProcessImage(const cv::Mat& raw, cv::Mat& preProcessed);
void removePerspective(const cv::Mat& input, cv::Mat& rectified);
int whiteThreshold_ = 149;
int undistortType_ = 1;
cv::Mat distortionMap1_;
cv::Mat distortionMap2_;

/* END OF PRE PROCEsSING*/

BlobDetection::BlobDetection() {
    // foundCircles initialisation
    // Maximum of 20 circles detected at the same time
    // It allows to loose a circle for a few frames without loosing its
    // information
    for (int j = 0; j < 20; j++) {
        foundCircles.emplace_back(RobotDesc(j, 0, 0));
    }
    // Init image
    trackbarsWhiteMat_ = Mat3b(1, 300, Vec3b(0, 0, 0));
    trackbarsRedMat_ = Mat3b(1, 300, Vec3b(0, 0, 0));
    trackbarsGreenMat_ = Mat3b(1, 300, Vec3b(0, 0, 0));
    maxID = 0;

    const std::string windowName_ = "Calib Circle Radius";
    cv::namedWindow(windowName_, CV_GUI_NORMAL);
    cv::createTrackbar("Max", windowName_, &MAX_CIRCLE_RADIUS, 200);
    cv::createTrackbar("Min", windowName_, &MIN_CIRCLE_RADIUS, 200);

    /* ToDO: REMOVE*/
    _tfListener = &tfListener_;
    cv::Mat distortedCamera =
        (cv::Mat_<float>(3, 3) << 422.918640, 0.000000, 350.119451, 0.000000,
         423.121112, 236.380265, 0.000000, 0.000000, 1.000000);

    cv::Mat cameraDistortion = (cv::Mat_<float>(1, 5) << -0.321590, 0.089597,
                                0.001090, -0.000489, 0.000000);

    cv::Mat undistortedCamera = cv::getOptimalNewCameraMatrix(
        distortedCamera, cameraDistortion, cv::Size(640, 480), 0);

    // cv::initUndistortRectifyMap(distortedCamera, cameraDistortion, cv::Mat(),
    // undistortedCamera,
    //  cv::Size(640, 480), CV_32FC1, distortionMap1_, distortionMap2_);
}

void BlobDetection::detect(const cv::Mat& input, cv::Mat& output_w,
                           cv::Mat& output_r, cv::Mat& output_g,
                           cv::Mat& output,
                           std::vector<RobotDesc>& robotsArray) {
    detectColor(input, output_w, output_r, output_g, output, robotsArray);
    detectCircles(input, output_w, output_r, output_g, output, robotsArray);
}

// Color detection algorithm
void BlobDetection::detectColor(const cv::Mat& input, cv::Mat& output_w,
                                cv::Mat& output_r, cv::Mat& output_g,
                                cv::Mat& output,
                                std::vector<RobotDesc>& robotsArray) {
    // Check if the input is empty
    if (!input.data)
        cerr << "Input of detecRobots is empty";  // TODO:throw an exception

    // Initialisation
    auto oldObjects = blobObjects;

    currentImage = input;

    // Blob detection with color thresholds
    output_w = whiteColor_.generateThreshold(currentImage);
    whiteColor_.trackFilteredObjects(currentImage);
    output_r = redColor_.generateThreshold(currentImage);
    redColor_.trackFilteredObjects(currentImage);
    output_g = greenColor_.generateThreshold(currentImage);
    greenColor_.trackFilteredObjects(currentImage);

    // extraction of white, red and green objects (robots)
    whiteObjects = whiteColor_.getObjects();
    greenObjects = greenColor_.getObjects();
    redObjects = redColor_.getObjects();

    for (auto object : greenObjects) {
        object.setColor(GREEN);
        object.setWindow(RotatedRect(
            Point2f(object.getXPos(), object.getYPos()),
            Size2f(sqrt(object.getArea()), sqrt(object.getArea())), 0));
        // set ID
        bool found = false;
        for (auto old : blobObjects) {
            double old_ray = sqrt(old.getArea() / PI);
            double distance = sqrt(abs(object.getXPos() - old.getXPos()) +
                                   abs(object.getYPos() - old.getYPos()));
            object.setDistance(distance);
            // if(distance < old_ray && !old.getAlreadyFound()){
            if (object.getXPos() < old.getXPos() + old_ray &&
                object.getXPos() > old.getXPos() - old_ray &&
                object.getYPos() < old.getYPos() + old_ray &&
                object.getYPos() > old.getYPos() - old_ray &&
                !old.getAlreadyFound() && old.getColor() == object.getColor()) {
                found = true;
                object.setID(old.getID());
                old.setAlreadyFound(true);
            }
        }
        if (!found) {
            object.setID(++maxID);
        }
        robotsArray.emplace_back(object);
    }
    for (auto object : redObjects) {
        object.setColor(RED);
        object.setWindow(RotatedRect(
            Point2f(object.getXPos(), object.getYPos()),
            cv::Size2f(sqrt(object.getArea()), sqrt(object.getArea())), 0));
        // set IDColor
        bool found = false;
        for (auto old : blobObjects) {
            double old_ray = sqrt(old.getArea() / PI);
            double distance = sqrt(abs(object.getXPos() - old.getXPos()) +
                                   abs(object.getYPos() - old.getYPos()));
            object.setDistance(distance);
            // if(distance < old_ray && !old.getAlreadyFound()){
            if (object.getXPos() < old.getXPos() + old_ray &&
                object.getXPos() > old.getXPos() - old_ray &&
                object.getYPos() < old.getYPos() + old_ray &&
                object.getYPos() > old.getYPos() - old_ray &&
                !old.getAlreadyFound() && old.getColor() == object.getColor()) {
                found = true;
                object.setID(old.getID());
                old.setAlreadyFound(true);
            }
        }
        if (!found) {
            object.setID(++maxID);
        }
        robotsArray.emplace_back(object);
    }
    blobObjects = robotsArray;

    // conversion of the input
    output = input;
}

// Circle detection algorithm
void BlobDetection::detectCircles(const cv::Mat& input, cv::Mat& output_w,
                                  cv::Mat& output_r, cv::Mat& output_g,
                                  cv::Mat& output,
                                  std::vector<RobotDesc>& robotsArray) {
    // Check if the input is empty
    if (!input.data)
        cerr << "Input of detecRobots is empty";  // TODO:throw an exception

    removePerspective(output, output);

    cv::Mat src_gray;
    cv::cvtColor(output, src_gray, CV_BGR2GRAY);

    vector<Vec3f> circles;

    tf::StampedTransform sTrans;
    tfListener_.lookupTransform("local_origin", "fcu", ros::Time(0), sTrans);

    cv::putText(output, "X: " + std::to_string(sTrans.getOrigin().x()),
                cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0,
                CV_RGB(255, 0, 0), 2.0);
    cv::putText(output, "Y: " + std::to_string(sTrans.getOrigin().y()),
                cv::Point(10, 65), cv::FONT_HERSHEY_PLAIN, 1.0,
                CV_RGB(0, 255, 0), 2.0);
    cv::putText(output, "Z: " + std::to_string(sTrans.getOrigin().z()),
                cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1.0,
                CV_RGB(0, 0, 255), 2.0);
    cv::putText(output, "MIN / MAX: " + std::to_string(MIN_CIRCLE_RADIUS) +
                            " / " + std::to_string(MAX_CIRCLE_RADIUS),
                cv::Point(10, 95), cv::FONT_HERSHEY_PLAIN, 1.0,
                CV_RGB(0, 0, 255), 2.0);
    cv::putText(output, "Robots Count: " + std::to_string(robotsArray.size()),
                cv::Point(10, 110), cv::FONT_HERSHEY_PLAIN, 1.0,
                CV_RGB(0, 180, 150), 2.0);

    double droneHeight = sTrans.getOrigin().z() -
                         0.14;  // 0.14 =~ distance lorsque le drone est au sol.

    cv::GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);
    /// Apply the Hough Transform to find the circles
    cv::HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8,
                     200, 70, MIN_CIRCLE_RADIUS,
                     MAX_CIRCLE_RADIUS * (1.0 - droneHeight));

    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        cv::circle(output, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }

    cv::waitKey(10);  // FOR TRACKBARS (TODO: remove)
}

void BlobDetection::createTrackbars() {
    // create windows for trackbars
    string WhiteTrackbars = "White Calibration Trackbars";
    string RedTrackbars = "Red Calibration Trackbars";
    string GreenTrackbars = "Green Calibration Trackbars";

    namedWindow(WhiteTrackbars, 0);
    namedWindow(RedTrackbars, 1);
    namedWindow(GreenTrackbars, 2);

    startWindowThread();

    whiteColor_.createTrackbars(WhiteTrackbars);
    redColor_.createTrackbars(RedTrackbars);
    greenColor_.createTrackbars(GreenTrackbars);

    imshow(WhiteTrackbars, trackbarsWhiteMat_);
    imshow(RedTrackbars, trackbarsRedMat_);
    imshow(GreenTrackbars, trackbarsGreenMat_);
}
void BlobDetection::saveCalibration(string filename) {
    std::fstream fs;
    fs.open(filename, std::fstream::out | std::fstream::trunc);
    if (!fs.fail()) {
        fs << *whiteColor_.H_MIN << endl;
        fs << *whiteColor_.H_MAX << endl;
        fs << *whiteColor_.S_MIN << endl;
        fs << *whiteColor_.S_MAX << endl;
        fs << *whiteColor_.V_MIN << endl;
        fs << *whiteColor_.V_MAX << endl;
        fs << *whiteColor_.PRE_EROSIONS << endl;
        fs << *whiteColor_.DILATIONS << endl;
        fs << *whiteColor_.POST_EROSIONS << endl;

        fs << *redColor_.H_MIN << endl;
        fs << *redColor_.H_MAX << endl;
        fs << *redColor_.S_MIN << endl;
        fs << *redColor_.S_MAX << endl;
        fs << *redColor_.V_MIN << endl;
        fs << *redColor_.V_MAX << endl;
        fs << *redColor_.PRE_EROSIONS << endl;
        fs << *redColor_.DILATIONS << endl;
        fs << *redColor_.POST_EROSIONS << endl;

        fs << *greenColor_.H_MIN << endl;
        fs << *greenColor_.H_MAX << endl;
        fs << *greenColor_.S_MIN << endl;
        fs << *greenColor_.S_MAX << endl;
        fs << *greenColor_.V_MIN << endl;
        fs << *greenColor_.V_MAX << endl;
        fs << *greenColor_.PRE_EROSIONS << endl;
        fs << *greenColor_.DILATIONS << endl;
        fs << *greenColor_.POST_EROSIONS << endl;

        fs.close();
    }
}
void BlobDetection::loadCalibration(string filename) {
    std::fstream fs;
    fs.open(filename, std::fstream::in);

    if (!fs.fail()) {
        fs >> *whiteColor_.H_MIN;
        fs >> *whiteColor_.H_MAX;
        fs >> *whiteColor_.S_MIN;
        fs >> *whiteColor_.S_MAX;
        fs >> *whiteColor_.V_MIN;
        fs >> *whiteColor_.V_MAX;
        fs >> *whiteColor_.PRE_EROSIONS;
        fs >> *whiteColor_.DILATIONS;
        fs >> *whiteColor_.POST_EROSIONS;

        fs >> *redColor_.H_MIN;
        fs >> *redColor_.H_MAX;
        fs >> *redColor_.S_MIN;
        fs >> *redColor_.S_MAX;
        fs >> *redColor_.V_MIN;
        fs >> *redColor_.V_MAX;
        fs >> *redColor_.PRE_EROSIONS;
        fs >> *redColor_.DILATIONS;
        fs >> *redColor_.POST_EROSIONS;

        fs >> *greenColor_.H_MIN;
        fs >> *greenColor_.H_MAX;
        fs >> *greenColor_.S_MIN;
        fs >> *greenColor_.S_MAX;
        fs >> *greenColor_.V_MIN;
        fs >> *greenColor_.V_MAX;
        fs >> *greenColor_.PRE_EROSIONS;
        fs >> *greenColor_.DILATIONS;
        fs >> *greenColor_.POST_EROSIONS;

        fs.close();
    }
}

std::string BlobDetection::getAllParams() {
    std::stringstream ss;

    ss << *redColor_.H_MAX << "\t" << *redColor_.H_MIN << "\t"
       << *redColor_.S_MAX << "\t" << *redColor_.S_MIN << "\t"
       << *redColor_.V_MAX << "\t" << *redColor_.V_MIN << "\t"
       << *redColor_.PRE_EROSIONS << "\t" << *redColor_.DILATIONS << "\t"
       << *redColor_.POST_EROSIONS << "\t"

       << *greenColor_.H_MAX << "\t" << *greenColor_.H_MIN << "\t"
       << *greenColor_.S_MAX << "\t" << *greenColor_.S_MIN << "\t"
       << *greenColor_.V_MAX << "\t" << *greenColor_.V_MIN << "\t"
       << *greenColor_.PRE_EROSIONS << "\t" << *greenColor_.DILATIONS << "\t"
       << *greenColor_.POST_EROSIONS << "\t"

       << *whiteColor_.H_MAX << "\t" << *whiteColor_.H_MIN << "\t"
       << *whiteColor_.S_MAX << "\t" << *whiteColor_.S_MIN << "\t"
       << *whiteColor_.V_MAX << "\t" << *whiteColor_.V_MIN << "\t"
       << *whiteColor_.PRE_EROSIONS << "\t" << *whiteColor_.DILATIONS << "\t"
       << *whiteColor_.POST_EROSIONS;

    return ss.str();
}

Eigen::Matrix4f BlobDetection::getPerspectiveProjectionTransform(
    double focalLength, double width, double height) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
    m(0, 0) = 2 * focalLength / width;
    m(1, 1) = 2 * focalLength / height;
    m(3, 2) = -1;

    return m;
}
void BlobDetection::removePerspective(const cv::Mat& input,
                                      cv::Mat& rectified) {
    double roll, pitch, yaw = 0.0;
    Eigen::Vector3f direction;
    try {
        tf::StampedTransform tf;
        tfListener_.lookupTransform("local_origin", "fcu", ros::Time(0), tf);

        tf::Matrix3x3 m(tf.getRotation());
        m.getRPY(roll, pitch, yaw);

        tf::Vector3 v = m * tf::Vector3(0.0, 0.0, 1.0);
        direction.x() = v.x();
        direction.y() = v.y();
        direction.z() = v.z();
    } catch (tf::TransformException e) {
        ROS_ERROR("%s", e.what());
    }

    Eigen::Matrix3f r = (Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(-roll, Eigen::Vector3f::UnitY()))
                            .toRotationMatrix();

    Eigen::Matrix4f R = Eigen::Matrix4f::Zero();
    R(3, 3) = 1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; j++) {
            R(i, j) = r(i, j);
        }
    }

    double height = input.size().height;
    double width = input.size().width;
    double f = 423.0;
    double HFOV = std::atan(width / (2 * f));
    double VFOV = std::atan(height / (2 * f));

    Eigen::Vector4f src[4]{Eigen::Vector4f(1.0, 1.0, 0.0, 1.0),
                           Eigen::Vector4f(-1.0, 1.0, 0.0, 1.0),
                           Eigen::Vector4f(-1.0, -1.0, 0.0, 1.0),
                           Eigen::Vector4f(1.0, -1.0, 0.0, 1.0)};

    Eigen::Vector4f dst[4]{Eigen::Vector4f(1.0, 1.0, 0.0, 1.0),
                           Eigen::Vector4f(-1.0, 1.0, 0.0, 1.0),
                           Eigen::Vector4f(-1.0, -1.0, 0.0, 1.0),
                           Eigen::Vector4f(1.0, -1.0, 0.0, 1.0)};

    Eigen::Matrix4f P = getPerspectiveProjectionTransform(f, width, height);
    Eigen::Translation<float, 4> T(Eigen::Vector4f(0.0, 0.0, -1.0, 0.0));

    for (int i = 0; i < 4; ++i) {
        dst[i] = T * dst[i];
        dst[i] = P * dst[i];
        dst[i] /= dst[i][3];

        src[i] = R * src[i];
        src[i] = T * src[i];
        src[i] = P * src[i];
        // src[i] = P * T * R * dst[i];
        src[i] /= src[i][3];
    }

    cv::Point2f tSrc[4], tDst[4];
    for (int i = 0; i < 4; ++i) {
        tSrc[i] = cv::Point2f(src[i].x() * width / 2.0 + width / 2.0,
                              src[i].y() * height / 2.0 + height / 2.0);
        tDst[i] = cv::Point2f(dst[i].x() * width / 2.0 + width / 2.0,
                              dst[i].y() * height / 2.0 + height / 2.0);
    }

    cv::Mat perspectiveTransform = cv::getPerspectiveTransform(tSrc, tDst);
    cv::warpPerspective(input, rectified, perspectiveTransform, input.size());

    // rectified = input.clone();
    for (int i = 0; i < 4; ++i) {
        cv::circle(rectified, tSrc[i], 5, cv::Scalar(0, 200, 0), -1);
        cv::circle(rectified, tDst[i], 5, cv::Scalar(0, 100, 0), -1);
    }
}

void BlobDetection::preProcessImage(const cv::Mat& raw, cv::Mat& preProcessed) {
    // undistort
    // Blur
    cv::Mat typeConverted = raw;
    // if (raw.type() != CV_8UC1)
    // {
    //     cerr<<"type img:"<<raw.type()<<" | GoodType: "<<CV_8UC1<<endl;
    //     cv::cvtColor(raw, typeConverted, CV_BGR2GRAY);
    // }
    // else
    // {
    //     raw.copyTo(typeConverted);
    // }

    cv::Mat undistorted = typeConverted;
    // if (!undistortType_)
    // {
    //     cv::remap(typeConverted, undistorted, distortionMap1_,
    //     distortionMap2_, CV_INTER_LINEAR);
    // }

    cv::Mat perspective;
    removePerspective(undistorted, perspective);
    // cv::imshow("undistorted", undistorted);
    // cv::imshow("perspective", perspective);

    cv::Mat blured;
    cv::GaussianBlur(perspective, blured, cv::Size(7, 7), 8, 8);

    cv::Mat eroded;
    cv::Mat element =
        cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::erode(blured, eroded, element, cv::Point(0), 8);

    cv::Mat thresholded;
    cv::threshold(blured, thresholded, whiteThreshold_, 255, CV_THRESH_BINARY);

    preProcessed = thresholded;

    // cv::imshow("PreProcessed", preProcessed);
}
