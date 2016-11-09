
#include "PerspectiveTransform.h"
#include "Line.h"
#include "LineGroup.h"
#include "RANSAC.h"
#include "Eigen/Core"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace localization
{

using Vector = Eigen::Vector2f;

void PerspectiveTransform::perspectiveTransformFromLines(const std::vector<Line>& lines)
{
    Vector orientations[2];
    RANSAC::findBestOrientationSplit(lines, 100, orientations);

    //Eigen::Vector2f point = RANSAC::findBestConvergencePoint(testLines, 10);
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

}