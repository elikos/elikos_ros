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

    inline const std::vector<Line*>& getLines() const;
    inline const cv::Vec2f& getAvgOrientation() const;

private :
    cv::Vec2f avgOrientation_;
    std::vector<Line*> lines_;

    LineGroup() = default;
};

inline const std::vector<Line*>& LineGroup::getLines() const
{
   return lines_;
}

inline const cv::Vec2f& LineGroup::getAvgOrientation() const
{
   return avgOrientation_;
}

}

#endif //LOCALIZATION_LINEGROUP_H
