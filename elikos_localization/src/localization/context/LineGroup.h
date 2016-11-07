//
// Created by olivier on 10/5/16.
//

#ifndef LOCALIZATION_LINEGROUP_H
#define LOCALIZATION_LINEGROUP_H

#include <Eigen/Core>

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
    inline const Eigen::Vector2f& getAvgOrientation() const;
    inline double getAvgRho() const;

private :
    Eigen::Vector2f avgOrientation_;
    float avgRho_;

    std::vector<Line*> lines_;

    LineGroup() = default;
};

inline const std::vector<Line*>& LineGroup::getLines() const
{
   return lines_;
}

inline const Eigen::Vector2f& LineGroup::getAvgOrientation() const
{
   return avgOrientation_;
}

inline double LineGroup::getAvgRho() const
{
   return avgRho_;
}

}

#endif //LOCALIZATION_LINEGROUP_H
