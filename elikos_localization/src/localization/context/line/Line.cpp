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

Line::Line(float rho, float theta, const Eigen::AlignedBox<float, 2>& frame)
    : Line(rho, theta)
{

    Eigen::Vector2f center = { (frame.min().x() + frame.max().x()) / 2.0,
                               (frame.min().y() + frame.max().y()) / 2.0 };

    Eigen::Vector2f dc = center - centroid_;

    double proj = dc.dot(orientation_);
    centroid_ += proj * orientation_;
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

bool Line::isCollateral(const Line& line, double threshold) const
{
    return std::abs(line.getOrientation().dot(orientation_)) < threshold;
}

void Line::draw(cv::Mat& image) const
{
    cv::Point2f pt1, pt2;
    pt1.x = cvRound(centroid_.x() + 1000 * (orientation_.x()));
    pt1.y = cvRound(centroid_.y() + 1000 * (orientation_.y()));
    pt2.x = cvRound(centroid_.x() - 1000 * (orientation_.x()));
    pt2.y = cvRound(centroid_.y() - 1000 * (orientation_.y()));
    // cv::line(image, pt1, pt2, cv::Scalar(100, 100, 100), 1, CV_AA);
    cvLine((IplImage*)(&image), pt1, pt2, cv::Scalar(100, 100, 100), 1, CV_AA,0);
    //void cvLine(CvArr*, CvPoint, CvPoint, CvScalar, int, int, int)
}

};
