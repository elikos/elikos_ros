
#include "PerspectiveTransform.h"
#include "Line.h"
#include "RANSAC.h"
#include "Eigen/Core"

namespace localization
{

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

}