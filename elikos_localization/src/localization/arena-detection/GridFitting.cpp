#include "GridFitting.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

namespace localization
{

using Vector = Eigen::Vector2f;

Grid GridFitting::findBestGridModel(const std::vector<Eigen::Vector2f>& intersections)
{
    debug++;
    if (debug == 16) {
        int t = 0;
    }
    // copy dataset into pointcloud.
    pcl::PointCloud<pcl::PointXY>::Ptr pc(new pcl::PointCloud<pcl::PointXY>());
    for (int i = 0; i < intersections.size(); ++i) 
    {
        pc->push_back({ intersections[i].x(), intersections[i].y() });
    } 

    pcl::KdTreeFLANN<pcl::PointXY> tree;
    tree.setInputCloud(pc);
    std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>& points = pc->points;

    double bestWeight = 10000000;
    Grid bestGrid;
    for (int i = 0; i < intersections.size(); ++i) 
    {
        // Search for a model;
        std::vector<int> neighborIndices(2);
        std::vector<float> distances(2);
        tree.nearestKSearch(points[i], 2, neighborIndices, distances); 

        pcl::PointXY p = points[neighborIndices[neighborIndices.size() - 1]];
        Grid grid(Vector(points[i].x, points[i].y), Vector(p.x, p.y));
        double weight = grid.weightIntersectionsFitness(intersections, { 320, 240 });

        if (weight < bestWeight) 
        {
            bestWeight = weight;
            bestGrid = grid;
        }
    }
    return bestGrid;
}

}