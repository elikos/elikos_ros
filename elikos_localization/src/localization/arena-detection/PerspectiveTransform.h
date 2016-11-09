#ifndef PERSPECTIVE_TRANSFORM_H
#define PERSPECTIVE_TRANSFORM_H

#include <vector>
#include <Eigen/Core>

namespace localization
{

class Line;
class LineGroup;

class PerspectiveTransform
{
public:
    PerspectiveTransform() = default;

    void perspectiveTransformFromLines(const std::vector<Line>& lines);

private:

    void splitLinesByOrientation(const std::vector<Line>& lines, Eigen::Vector2f orientations[2], LineGroup groups[2]);

};

}

#endif // PERSPECTIVE_TRANSFORM_H