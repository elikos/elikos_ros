#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include <Eigen/Dense>

namespace localization {
    class Line;
}

namespace RANSAC
{

Eigen::Vector2f findBestConvergencePoint(const std::vector<localization::Line>& lines, int k);
float summConvergeDistance(const std::vector<localization::Line>& lines, const localization::Line& pivot, const Eigen::Vector2f& convergencePoint);

bool findBestOrientationSplit(const std::vector<localization::Line>& lines, int k, Eigen::Vector2f bestOrientations[2]);
float evaluateOrientationModel(const std::vector<localization::Line>& lines, Eigen::Vector2f orientations[2]);

}

#endif // RANSAC_H