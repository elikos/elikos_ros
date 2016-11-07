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

}

#endif // RANSAC_H