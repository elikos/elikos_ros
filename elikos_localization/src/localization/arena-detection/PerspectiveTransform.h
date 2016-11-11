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

    void splitLinesByOrientation(const std::vector<Line>& lines, Eigen::Vector2f orientations[2], LineGroup groups[2]);
    Eigen::Vector2f translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation);
    Eigen::Vector2f rotate(const Eigen::Vector2f& v, double theta);

};

}

#endif // PERSPECTIVE_TRANSFORM_H