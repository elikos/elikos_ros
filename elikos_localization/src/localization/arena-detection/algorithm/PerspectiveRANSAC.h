#include <pcl/point_types.h>
#include <vector>

#ifndef PERSPECTIVE_RANSAC
#define PERSPECTIVE_RANSAC

namespace RANSAC
{

void findBestConvergencePoint(const std::vector<pcl::PointXY>& points);

}

#endif