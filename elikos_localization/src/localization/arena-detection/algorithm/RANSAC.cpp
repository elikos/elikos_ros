#include "RANSAC.h"

#include "Line.h"
#include <opencv2/highgui/highgui.hpp>

namespace RANSAC
{

using Vector = Eigen::Vector2f;
using Line = localization::Line;

Eigen::Vector2f findBestConvergencePoint(const std::vector<Line>& lines, int k)
{
    for (int i = 0; i < k; ++i) {
        
    }
}

double sumIntersectDistToPivotLine(const std::vector<Line>& lines, const Line& pivot, const Vector& convergence)
{
    /*
    double distance = 0.0;
    for (int i = 0; i < lines.size(); ++i) {
        cv::Vec2f intersection;
        bool intersects = lines[i].findIntersection(pivot, intersection);
        if (intersects) 
            Point distance = 
            distance += convergence.distance({ convergence.x, convergence.y });
        }
    } 
    return distance;
    */
}


}