#ifndef INTERSECTION_TRANSFORM_H
#define INTERSECTION_TRANSFORM_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization
{

class QuadState;

class IntersectionTransform
{
public:

    IntersectionTransform(double focalLength, QuadState* state);
    ~IntersectionTransform() = default;

    void transformIntersections(const std::vector<Eigen::Vector2f>& imageIntersections) const;

private:

    double estimateAltitude(const std::Vector<Eigen::Vector2f>& imageIntersections) const;
    void transformIntersectionXY(const Eigen::Vector2f& imageIntersection, Eigen::Vector3f& intersection) const;
    void publishTransformedIntersections(const std::vector<Eigen::Vector3f>& intersections) const;

    const double focalLength_;
    QuadState* const state_;

    pcl::KdTreeFLANN<pcl::PointXY> kdTree_;
    pcl::PointCloud<pck::PointXY> pointCloud_;

    pcl::PointCloud<pcl::PointXY>::Ptr pc(new pcl::PointCloud<pcl::PointXY>());
    for (int i = 0; i < dataset.size(); ++i) {
        pc->push_back({ dataset[i].x(), dataset[i].y() });
    } 
};

}

#endif // INTERSECTION_TRANSFORM_H