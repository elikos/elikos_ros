#include "RANSAC.h"

#include "Line.h"
#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>

#include <ctime>

namespace RANSAC
{

using Vector = Eigen::Vector2f;
using Line = localization::Line;

Vector findBestConvergencePoint(const std::vector<Line>& lines, int k)
{
    std::srand(std::time(NULL));
    float bestDistance = std::numeric_limits<float>::max();
    Vector bestConvergencePoint(0.0, 0.0);
    for (int i = 0; i < k; ++i) {
        const Line& firstLine = lines[std::rand() % lines.size()];
        const Line& otherLine = lines[std::rand() % lines.size()];

        Vector intersection;
        bool isIntersecting = firstLine.findIntersection(otherLine, intersection);
        if (isIntersecting) {
            double distance = summConvergeDistance(lines, firstLine, intersection);
            distance += summConvergeDistance(lines, otherLine, intersection);
            if (distance < bestDistance) {
                bestConvergencePoint = intersection; 
                bestDistance = distance;
            }
        }
        // else handle parallel lines here.
    }
    return bestConvergencePoint;
}

float summConvergeDistance(const std::vector<Line>& lines, const Line& pivot, const Vector& convergence)
{
    float distance = 0.0;
    for (int i = 0; i < lines.size(); ++i) {
        Vector intersection;
        bool isIntersecting = lines[i].findIntersection(pivot, intersection);
        if (isIntersecting) {
            distance += (intersection - convergence).squaredNorm();
        }
    } 
    return distance;
}

}