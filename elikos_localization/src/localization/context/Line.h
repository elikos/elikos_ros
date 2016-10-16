//
// Created by olivier on 10/2/16.
//

#ifndef LOCALIZATION_ARENA_LINE_H
#define LOCALIZATION_ARENA_LINE_H

#include <opencv2/highgui/highgui.hpp>

namespace localization
{

class Line
{
public:
    Line(float rho, float theta);
    Line(float rho, const cv::Vec2d& orientation);

    void inverseOrientation();
    void rotate(double rotation);
    bool findIntersection(const Line& line, cv::Point2d& intersection) const;

    inline float getRho() const;
    inline cv::Vec2d getOrientation() const;
    inline cv::Point2d getCentroid() const;

private:
    float rho_;
    cv::Vec2d orientation_;
    cv::Point2d centroid_;

    Line() = default;
};

inline float Line::getRho() const
{
   return rho_;
}

inline cv::Vec2d Line::getOrientation() const
{
    return orientation_;
};

inline cv::Point2d Line::getCentroid() const
{
    return centroid_;
}

}

#endif //ELIKOS_LOCALIZATION_ARENALINE_H
