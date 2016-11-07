#include "RANSAC.h"

#include "Line.h"
#include <opencv2/highgui/highgui.hpp>

namespace RANSAC
{

pcl::PointXY findBestConvergencePoint(const std::vector<localization::Line>& lines, int k)
{
    
    for (int i = 0; i < k; ++i) {
    }
}

double sumIntersectDistToPivotLine(const std::vector<localization::Line>& lines, localization::Line pivot, pcl::PointXY convergencePoint)
{
    double distance = 0.0;
    for (int i = 0; i < lines.size(); i++) {
        cv::Point2f intersection;
        bool intersects = lines[i].findIntersection(pivot, intersection);
        if (intersects) {
            distance += pcl::PointXY(intersection.x, intersection.y).distance({convergencePoint.x, convergencePoint.y});
        }
    } 
    return distance;
}


}