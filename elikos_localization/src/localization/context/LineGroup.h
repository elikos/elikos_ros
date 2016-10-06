//
// Created by olivier on 10/5/16.
//

#ifndef LOCALIZATION_LINEGROUP_H
#define LOCALIZATION_LINEGROUP_H

#include <opencv2/core/core.hpp>

namespace localization
{

class LineGroup
{
public:
    LineGroup(cv::Vec2f& v);
    void add(cv::Vec2f& v);

    inline const std::vector<cv::Vec2f*> &getLines() const;
    inline const cv::Vec2f& getAvgLine() const;

private :
    cv::Vec2f avg_;
    std::vector<cv::Vec2f*> lines_;

    LineGroup() = default;
};

inline const std::vector<cv::Vec2f*>& LineGroup::getLines() const
{
   return lines_;
}

inline const cv::Vec2f& LineGroup::getAvgLine() const
{
   return avg_;
}

}

#endif //LOCALIZATION_LINEGROUP_H
