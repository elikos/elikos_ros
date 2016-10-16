//
// Created by olivier on 10/2/16.
//

#include "Line.h"

namespace localization
{

Line::Line(float rho, float theta)
    : rho_(std::abs(rho))
{
    orientation_ = cv::Vec2d(sinf(theta), cosf(theta));
    centroid_ = cv::Point2d(rho * orientation_[1], rho * orientation_[0]);
}

Line::Line(float rho, const cv::Vec2d& orientation)
    : rho_(std::abs(rho)), orientation_(orientation)
{
    centroid_ = cv::Point2d(rho * orientation_[1], rho * orientation_[0]);
}

void Line::rotate(double rotation)
{
    double x = orientation_[0];
    double y = orientation_[1];

    orientation_[0] = x * cos(rotation) - y * sin(rotation);
    orientation_[1] = x * sin(rotation) + y * cos(rotation);
}

void Line::inverseOrientation()
{
    rotate(CV_PI);
}


bool Line::findIntersection(const Line& otherLine, cv::Point2d& intersection) const
{
    const int x = 0;
    const int y = 1;


    //cv::Point2f A = { 100.0, 100.0 };
    //cv::Point2f B = { 200.0, 200.0};

    cv::Point2d A = centroid_;
    cv::Point2d B = otherLine.centroid_;

    cv::Vec2d dc = B - A;

    //cv::Vec2f u = { 2.0, 1.0 };
    //cv::Vec2f v = { -1.0, -2.0 };

    cv::Vec2d u = orientation_;
    cv::Vec2d v = otherLine.orientation_;

    u = cv::normalize(u);
    v = cv::normalize(v);

    // Convert the coordinate system (origin -> top-left to bottom-left)
    u[1] = -u[1];
    v[1] = -v[1];

    double det =  u[x] * v[y] - u[y] * v[x];

    if (det != 0.0) {

        double d = (dc[x] * v[y] - dc[y] * v[x]) / det;
        intersection = A + (cv::Point2d)(d * u);

    } else {
        return false;
    }
    return true;
}

};
