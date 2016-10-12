//
// Created by olivier on 10/2/16.
//

#include "Line.h"

namespace localization
{

Line::Line(float rho, float theta)
    : rho_(rho), theta_(theta)
{
    orientation_ = cv::Vec2f(sinf(theta), cosf(theta));
    centroid_ = cv::Point2f(rho * orientation_[1], rho * orientation_[0]);
}

void Line::rotate(double rotation)
{
    theta_ += rotation;
    orientation_ = { sinf(theta_), cosf(theta_) };
}

void Line::inverseOrientation()
{
    theta_ += CV_PI;
    orientation_ = { sinf(theta_), cosf(theta_) };
}


bool Line::findIntersection(const Line& otherLine, cv::Point2f& intersection) const
{
    const int x = 0;
    const int y = 1;

    //cv::Vec2f dc = { 2.0, 0.0 };
    //cv::Point2f A = { 0.0, 0.0 };
    //cv::Vec2f u = {1.0, 1.0};
    //cv::Vec2f v = {-1.0, 1.0};

    cv::Point2f A = centroid_;
    cv::Vec2f dc = otherLine.centroid_ - centroid_;
    cv::Vec2f u = orientation_;
    cv::Vec2f v = otherLine.orientation_;

    u = cv::normalize(u);
    v = cv::normalize(v);

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
