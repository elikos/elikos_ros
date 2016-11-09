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

bool findBestOrientationSplit(const std::vector<localization::Line>& lines, int k, Vector bestOrientations[2])
{
    std::srand(std::time(NULL));
    float bestFitWeight = 0.0;
    for (int i = 0; i < k; ++i) {
        const Line& line = lines[std::rand() % lines.size()];
        Vector orientation = line.getOrientation();

        Vector orientations[2] = { orientation, 
                                   { orientation.y(),
                                    -orientation.x() }};
        float fitWeight = evaluateOrientationModel(lines, orientations);
        if (fitWeight > bestFitWeight) {
            bestFitWeight = fitWeight;
            bestOrientations[0] = orientations[0];
            bestOrientations[1] = orientations[1];
        }
    }
    return true;
}

float evaluateOrientationModel(const std::vector<localization::Line>& lines, Eigen::Vector2f orientations[2])
{
    double dotSumm = 0.0;
    for (int i = 0; i < lines.size(); ++i) {
        Vector u = lines[i].getOrientation();

        double udotv = std::abs(u.dot(orientations[0]));
        double udotw = std::abs(u.dot(orientations[1]));

        bool isCloserToV = udotv > udotw;
        dotSumm += (isCloserToV) ? udotv : udotw;
        int iOrientation = (isCloserToV) ? 0 : 1;
    }
    return dotSumm;
}

}