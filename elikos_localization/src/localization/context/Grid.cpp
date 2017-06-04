#include "Grid.h"
#include "Eigen/Geometry"
#include <cmath>
#include "Line.h"

namespace localization 
{

using Vector = Eigen::Vector2f;

Grid::Grid(const Eigen::Vector2f& origin, const Eigen::Vector2f& neighbor)
{
    Eigen::Vector2f d = neighbor - origin;
    distance_ = d.norm();    
    direction_ = d.normalized();
    origin_ = origin;
}

double Grid::weightIntersectionsFitness(const std::vector<Eigen::Vector2f>& intersections,
                                        const Eigen::Vector2f& imageCenter)
{
    Eigen::Translation<float, 2> translation(imageCenter);
    Eigen::Translation<float, 2> invTrans(-imageCenter);

    Vector rotated(0.0 , 1.0);

    double dot = direction_.dot(rotated);
    double cross = std::abs(direction_.x() * rotated.y() - rotated.x() * direction_.y());

    // Build rotation matrix
    Eigen::Matrix2f rotation;
    rotation(0, 0) = dot;
    rotation(1, 0) = cross;
    rotation(0, 1) = -cross;
    rotation(1, 1) = dot;

    Eigen::Transform<float, 2, Eigen::Affine> T = translation * rotation * invTrans; 
    Vector rotatedOrigin = T * origin_;

    double weight = 0.0;
    for (size_t i = 0; i < intersections.size(); ++i) 
    {
        Vector p = T * intersections[i] - rotatedOrigin;
        p /= distance_;
        weight += std::abs(p.x() - std::round(p.x()));
        weight += std::abs(p.y() - std::round(p.y()));
    }

    int w = imageCenter.x() * 2;
    int h = imageCenter.y() * 2;
    double nExpected = (h * w) / (distance_ * distance_); 

    weight += std::abs(nExpected - intersections.size());
    

    return weight;
}

void Grid::draw(cv::Mat& image) const
{
    Vector directions[4] = {
        { direction_ },
        { -direction_.y(),  direction_.x() },
        {  direction_.y(), -direction_.x() },
        { -direction_.x(), -direction_.y() }};

    Eigen::AlignedBox<float, 2> box(Vector(0.0, 0.0), Vector(640.0, 480.0));

    for (size_t i = 0; i < 4; ++i)
    {
        Vector p = origin_;
        while (box.contains(p)) 
        {
            Vector direction = Vector(directions[i].y(), -directions[i].x());
            Line line(p, direction);
            line.draw(image);
            p += directions[i] * distance_;
        }
    }
}

}