#ifndef PERSPECTIVE_TRANSFORM_H
#define PERSPECTIVE_TRANSFORM_H

#include <vector>

namespace localization
{

class Line;

class PerspectiveTransform
{
public:
    PerspectiveTransform() = default;

    void perspectiveTransformFromLines(const std::vector<Line>& lines);

};

}

#endif // PERSPECTIVE_TRANSFORM_H