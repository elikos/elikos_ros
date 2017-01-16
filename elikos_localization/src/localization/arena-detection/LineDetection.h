#ifndef LOCALIZATION_LINE_DETECTION_H
#define LOCALIZATION_LINE_DETECTION_H

#include <vector>

#include "Line.h"

namespace localization
{

class LineDetection
{
public:

    using Vector = Eigen::Vector2f;

    LineDetection() = default;
    ~LineDetection() = default;

    void filterLineCluster(const std::vector<Line>& cluster);
    inline const std::vector<Line>& getFilteredLines() const;

private:

    std::vector<Line> filteredLines_;

    void filterByOrientation(const std::vector<Line>& cluster, std::vector<LineGroup>& filtered);

    void groupByOrientation(const Line& line, std::vector<LineGroup>& groups);

    void filterByProximity(const LineGroup& group, std::vector<LineGroup>& filtered);

    void parseClusterMemberships(const std::vector<int>& clusterMemberships, 
                                 const std::vector<const Line*>& cluster,
                                       std::vector<LineGroup>& filtered);

};

inline const std::vector<Line>& LineDetection::getFilteredLines() const
{
    return filteredLines_;
}

}

#endif // LOCALIZATION_LINE_DETECTION_H