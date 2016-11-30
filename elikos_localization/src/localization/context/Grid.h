#ifndef GRID_H
#define GRID_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace localization
{

class Grid
{
public:
    Grid() = default;
    Grid(const Eigen::Vector2f& origin, const Eigen::Vector2f& neighbor);
    ~Grid() = default;

    inline double getDistance() const; 

    double weightIntersectionsFitness(const std::vector<Eigen::Vector2f>& intersections,
                                      const Eigen::Vector2f& imageCenter);

    void draw(cv::Mat& image) const;

private:

    Eigen::Vector2f direction_;
    Eigen::Vector2f origin_;
    double distance_;

};

inline double Grid::getDistance() const
{
    return distance_;
}

}

#endif // GRID_H