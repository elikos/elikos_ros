//
// Created by olivier on 10/2/16.
//

#include "Line.h"

namespace localization
{

Line::Line(float rho, float theta)
    : rho_(std::abs(rho))
{
    orientation_ = cv::Vec2f(sinf(theta), cosf(theta));
    centroid_ = cv::Point2f(rho * orientation_[1], rho * orientation_[0]);
}

Line::Line(float rho, const cv::Vec2f& orientation)
    : rho_(std::abs(rho)), orientation_(orientation)
{
    centroid_ = cv::Point2f(rho * orientation_[1], rho * orientation_[0]);
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


bool Line::findIntersection(const Line& otherLine, cv::Point2f& intersection) const
{
    const int x = 0;
    const int y = 1;


    //cv::Point2f A = { 100.0, 100.0 };
    //cv::Point2f B = { 200.0, 200.0};

    cv::Point2f A = centroid_;
    cv::Point2f B = otherLine.centroid_;

    cv::Vec2f dc = B - A;

    //cv::Vec2f u = { 2.0, 1.0 };
    //cv::Vec2f v = { -1.0, -2.0 };

    cv::Vec2f u = orientation_;
    cv::Vec2f v = otherLine.orientation_;

    u = cv::normalize(u);
    v = cv::normalize(v);

    // Convert the coordinate system (origin -> top-left to bottom-left)
    u[1] = -u[1];
    v[1] = -v[1];

    double det =  u[x] * v[y] - u[y] * v[x];

    if (det != 0.0) {

        double d = (dc[x] * v[y] - dc[y] * v[x]) / det;
        intersection = A + (cv::Point2f)(d * u);

    } else {
        return false;
    }
    return true;
}

};
