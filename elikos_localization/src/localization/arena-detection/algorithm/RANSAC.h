#ifndef PERSPECTIVE_RANSAC
#define PERSPECTIVE_RANSAC

#include <vector>
#include <pcl/point_types.h>

namespace localization {
    class Line;
}

namespace RANSAC
{

pcl::PointXY findBestConvergencePoint(const std::vector<localization::Line>& lines, int k);
double sumIntersectDistToPivotLine(const std::vector<localization::Line>& lines, Line pivot, pcl::PointXY convergencePoint)
 

}

#endif