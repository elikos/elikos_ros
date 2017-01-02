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
    LineGroup(const Line& v);
    void add(const Line& v);

    Line convertToLine() const;

    inline const std::vector<const Line*>& getLines() const;
    inline const Eigen::Vector2f& getAvgOrientation() const;
    inline double getAvgRho() const;

    bool isCollateral(const Line& line, double threshold);

private :
    Eigen::Vector2f avgOrientation_;
    float avgRho_;
    Eigen::Vector2f avgCentroid_;
    float avgTheta_;

    std::vector<const Line*> lines_;

    LineGroup() = default;
};

inline const std::vector<const Line*>& LineGroup::getLines() const
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
