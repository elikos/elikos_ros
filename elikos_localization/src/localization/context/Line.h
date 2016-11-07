//
// Created by olivier on 10/2/16.
//

#ifndef LOCALIZATION_ARENA_LINE_H
#define LOCALIZATION_ARENA_LINE_H

#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>

namespace localization
{

class Line
{
public:
    Line(float rho, float theta);
    Line(float rho, const Eigen::Vector2f& orientation);
    Line(const Eigen::Vector2f& centroid, const Eigen::Vector2f& orientation);

    void inverseOrientation();
    void rotate(float rotation);
    bool findIntersection(const Line& line, Eigen::Vector2f& intersection) const;

    inline float getRho() const;
    inline float getTheta() const;
    inline Eigen::Vector2f getOrientation() const;
    inline Eigen::Vector2f getCentroid() const;

private:
    float rho_;
    float theta_;
    Eigen::Vector2f orientation_;
    Eigen::Vector2f centroid_;

    Line() = default;
};

inline float Line::getRho() const
{
   return rho_;
}

inline float Line::getTheta() const 
{
    return theta_;
}

inline Eigen::Vector2f Line::getOrientation() const
{
    return orientation_;
};

inline Eigen::Vector2f Line::getCentroid() const
{
    return centroid_;
}

}

#endif //ELIKOS_LOCALIZATION_ARENALINE_H
