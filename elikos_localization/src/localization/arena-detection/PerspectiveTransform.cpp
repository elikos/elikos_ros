
#include "PerspectiveTransform.h"
#include "Line.h"
#include "LineGroup.h"
#include "RANSAC.h"
#include "Eigen/Core"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>

namespace localization
{

using Vector = Eigen::Vector2f;

void PerspectiveTransform::perspectiveTransformFromLines(const std::vector<Line>& lines)
{
    std::vector<Line> testLines = {
        Line({0, 0}, {1, 1}),
        Line({1, 0}, {0, 1}),
        Line({2, 0}, {-1, 1}),
        Line({0,0}, {0, 1})
    };


    Eigen::Vector2f point = RANSAC::findBestConvergencePoint(testLines, 10);

    int test = 0;
}

void PerspectiveTransform::splitLinesByOrientation(const std::vector<Line>& lines, Vector orientations[2], LineGroup groups[2])
{
    for (int i = 0; i < lines.size(); ++i) {
        Vector u = lines[i].getOrientation();
        bool isFirstOrientationCloser = u.dot(orientations[0]) > u.dot(orientations[1]);
        LineGroup& group = (isFirstOrientationCloser) ? groups[0] : groups[1];
        group.add(lines[i]);
    } 
}

Eigen::Vector2f PerspectiveTransform::translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation)
{
    return v + translation;
}

Eigen::Vector2f PerspectiveTransform::rotate(const Eigen::Vector2f& v, double theta)
{
    double cosTheta = cosf(theta);
    double sinTheta = sinf(theta);

    Eigen::Vector2f rotated;
    rotated.x() = v.x() * cosTheta - v.y() * sinTheta;
    rotated.y() = v.x() * sinTheta + v.y() * cosTheta;

    return rotated;
}

}