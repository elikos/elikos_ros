//
// Created by olivier on 10/5/16.
//

#include "LineGroup.h"

namespace localization
{

LineGroup::LineGroup(cv::Vec2f& line)
{
    avg_ = line;
    lines_.push_back(&line);
}

void LineGroup::add(cv::Vec2f& line)
{
    avg_ *= (int)(lines_.size());
    lines_.push_back(&line);
    avg_ += line;
    avg_ /= (int)(lines_.size());
}

}
