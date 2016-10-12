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

    void inverseOrientation();
    void rotate(double rotation);
    bool findIntersection(const Line& line, cv::Point2f& intersection) const;

    inline float getRho() const;
    inline float getTheta() const;
    inline cv::Vec2f getOrientation() const;
    inline cv::Point2f getCentroid() const;

private:
    float rho_;
    float theta_;
    cv::Vec2f orientation_;
    cv::Point2f centroid_;

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
