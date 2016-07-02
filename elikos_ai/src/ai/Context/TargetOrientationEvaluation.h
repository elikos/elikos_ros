//
// Created by olivier on 01/07/16.
//

#ifndef AI_TARGET_ORIENTATION_EVALUATION_H
#define AI_TARGET_ORIENTATION_EVALUATION_H

#include <tf/tf.h>

namespace ai
{

class TargetOrientationEvaluation
{
public:
    TargetOrientationEvaluation() = default;
    ~TargetOrientationEvaluation() = default;

    inline void setGoodIntersection(bool goodIntersection);
    inline bool getGoodIntersection() const;

    inline void setLineIntersectionDistance(double distance);
    inline double getLineIntersectionDistance() const;

    inline void setOptimalOrientation(const tf::Vector3& orientation);
    inline const tf::Vector3& getOptimalOrientation() const;

    inline void setIntersectionPoint(const tf::Point intersection);
    inline const tf::Point& getIntersectionPoint() const;

private:
    bool goodIntersection_{ false };
    double lineIntersectionDistance_;
    tf::Point intersectionPoint_;
    tf::Vector3 optimalOrientation_;
};

inline void TargetOrientationEvaluation::setGoodIntersection(bool goodIntersection)
{
    goodIntersection_ = goodIntersection;
}

inline bool TargetOrientationEvaluation::getGoodIntersection() const
{
    return goodIntersection_;
}

inline void TargetOrientationEvaluation::setLineIntersectionDistance(double distance)
{
    lineIntersectionDistance_ = distance;
}

inline double TargetOrientationEvaluation::getLineIntersectionDistance() const
{
    return lineIntersectionDistance_;
}

inline void TargetOrientationEvaluation::setOptimalOrientation(const tf::Vector3& orientation)
{
    optimalOrientation_ = orientation;
}

inline const tf::Vector3& TargetOrientationEvaluation::getOptimalOrientation() const
{
    return optimalOrientation_;
}

inline void TargetOrientationEvaluation::setIntersectionPoint(const tf::Point intersection)
{
    intersectionPoint_ = intersection;
}

inline const tf::Point& TargetOrientationEvaluation::getIntersectionPoint() const
{
    return intersectionPoint_;
}

}

#endif // AI_TARGET_ORIENTATION_EVALUATION_H
