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
    Line(float rho, const cv::Vec2f& orientation);

    void inverseOrientation();
    void rotate(float rotation);
    bool findIntersection(const Line& line, cv::Point2f& intersection) const;

    inline float getRho() const;
    inline cv::Vec2f getOrientation() const;
    inline cv::Point2f getCentroid() const;

private:
    float rho_;
    cv::Vec2f orientation_;
    cv::Point2f centroid_;

    Line() = default;
};

inline float Line::getRho() const
{
   return rho_;
}

inline cv::Vec2f Line::getOrientation() const
{
    return orientation_;
};

inline cv::Point2f Line::getCentroid() const
{
    return centroid_;
}

}

#endif //ELIKOS_LOCALIZATION_ARENALINE_H
