//
// Created by olivier on 10/2/16.
//

#include "Line.h"

namespace localization
{

using Vector = Eigen::Vector2f;

Line::Line(float rho, float theta)
    : rho_(std::abs(rho)), theta_(theta)
{
    orientation_ = Vector(sinf(theta), -cosf(theta));
    centroid_ = Vector(rho * cos(theta), rho * sin(theta));
}

Line::Line(float rho, const Vector& orientation)
    : rho_(std::abs(rho)), orientation_(orientation.normalized())
{
    centroid_ = Vector(rho * orientation_[1], rho * orientation_[0]);
}

Line::Line(const Vector& centroid, const Vector& orientation)
    : centroid_(centroid), orientation_(orientation.normalized())
{
}

void Line::rotate(float rotation)
{
    float x = orientation_[0];
    float y = orientation_[1];

    orientation_[0] = x * cosf(rotation) - y * sinf(rotation);
    orientation_[1] = x * sinf(rotation) + y * cosf(rotation);
}

void Line::inverseOrientation()
{
    rotate(CV_PI);
}


bool Line::findIntersection(const Line& otherLine, Vector& intersection) const
{
    const int x = 0;
    const int y = 1;

    Vector A = centroid_;
    Vector B = otherLine.centroid_;

    Vector dc = B - A;

    Vector u = orientation_;
    Vector v = otherLine.orientation_;

    u.normalize();
    v.normalize();

    double det =  u[x] * v[y] - u[y] * v[x];

    if (det != 0.0) {

        double d = (dc[x] * v[y] - dc[y] * v[x]) / det;
        intersection = A + (d * u);

    } else {
        return false;
    }
    return true;
}

};
