//
// Created by olivier on 10/5/16.
//

#ifndef LOCALIZATION_LINEGROUP_H
#define LOCALIZATION_LINEGROUP_H

#include <opencv2/core/core.hpp>

namespace localization
{

class Line;

class LineGroup
{
public:
    LineGroup(Line& v);
    void add(Line& v);

    Line convertToLine() const;

    inline const std::vector<Line*>& getLines() const;
    inline const cv::Vec2d& getAvgOrientation() const;
    inline double getAvgRho() const;

private :
    cv::Vec2d avgOrientation_;
    double avgRho_;

    std::vector<Line*> lines_;

    LineGroup() = default;
};

inline const std::vector<Line*>& LineGroup::getLines() const
{
   return lines_;
}

inline const cv::Vec2d& LineGroup::getAvgOrientation() const
{
   return avgOrientation_;
}

inline double LineGroup::getAvgRho() const
{
   return avgRho_;
}

}

#endif //LOCALIZATION_LINEGROUP_H
