#ifndef PERSPECTIVE_RANSAC
#define PERSPECTIVE_RANSAC

#include <vector>
#include <Eigen/Dense>

namespace localization {
    class Line;
}

namespace RANSAC
{

Eigen::Vector2f findBestConvergencePoint(const std::vector<localization::Line>& lines, int k);
double sumIntersectDistToPivotLine(const std::vector<localization::Line>& lines, localization::Line pivot, Eigen::Vector2f convergencePoint);
 

}

#endif