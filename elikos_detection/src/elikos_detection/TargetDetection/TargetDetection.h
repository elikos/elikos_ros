//
// Created by ta11e4rand on 10/02/16.
//

#ifndef DETECTION_AND_TRACKING_TargetDetection_H
#define DETECTION_AND_TRACKING_TargetDetection_H

#include "BlobDetection.h"
#include "ShapeDetection.h"

#include <list>
class TargetDetection {
   public:
    TargetDetection();
    // Robot tracking algorithm. Includes color and robot detection
    void detect(const cv::Mat& input, cv::Mat& output_w, cv::Mat& output_r,
                cv::Mat& output_g, cv::Mat& output,
                std::vector<RobotDesc>& outputRobotsArray);

    // TODO: remove..
    void createTrackbars() { blobDetection_.createTrackbars(); }

    void saveCalibration(string filename) {
        blobDetection_.saveCalibration(filename);
    }
    void loadCalibration(string filename) {
        blobDetection_.loadCalibration(filename);
    }
    std::string getAllParams() { return blobDetection_.getAllParams(); }

    void updateHSV(int color, int h, int s, int v, int delta);
    void fetchRemoteParams(int color, int h_max, int h_min, int s_max,
                           int s_min, int v_max, int v_min, int preErode,
                           int dilate, int postErode);

   private:
    cv::Mat targetRobot_;
    void displayID(const RobotDesc& robot, cv::Mat& output) const;
    void displayRobots(const std::vector<RobotDesc>& robotsArray,
                       cv::Mat& output) const;

    /* Filters the duplicated RebotDesc from the vector and removes them.
     * Use
     * this method after running each detection type (after running at least
     * 2
     * detection types).
     *
     * Example sequence:
     *
     * runColorDetection --> runCircleDetection() -->
     * filterDuplicated(colorDetectionResult + circleDetection) -->
     * runAnotherDetection() --> filterDuplicated(filteredResults +
     * anotherDetectionResults)
     *
     */
    void filterDuplicated(std::vector<RobotDesc>& robotsArray) const;

    /* Detection classes */
    // Color
    BlobDetection blobDetection_;
    // Shape
    ShapeDetection shapeDetection_;
};

#endif  // DETECTION_AND_TRACKING_TargetDetection_H
