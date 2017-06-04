#include <unordered_map>
#include <iostream>

#include <Eigen/Core>

#include "DBSCAN.h"

#include "LineGroup.h"

#include "LineDetection.h"

namespace localization
{

void LineDetection::filterLineCluster(const std::vector<Line>& cluster)
{
    if (cluster.empty()) return;

    std::vector<LineGroup> orientationFiltered;
    filterByOrientation(cluster, orientationFiltered);

    std::vector<LineGroup> proximityFiltered;
    for (int i = 0; i < orientationFiltered.size(); ++i)
    {
        filterByProximity(orientationFiltered[i], proximityFiltered);
    }
    
    filteredLines_.clear();
    for (int i = 0; i < proximityFiltered.size(); ++i)
    {
        filteredLines_.push_back(proximityFiltered[i].convertToLine());
    }
}

void LineDetection::filterByOrientation(const std::vector<Line>& cluster, 
                                        std::vector<LineGroup>& filtered) 
{
    for (int i = 0; i < cluster.size(); ++i) 
    {
        groupByOrientation(cluster[i], filtered);
    }
} 

void LineDetection::groupByOrientation(const Line& line, std::vector<LineGroup>& groups)
{
    bool groupFound = false;
    for (int i = 0; i < groups.size() && !groupFound; ++i) 
    {
        if (groups[i].isCollateral(line, 0.80))
        {
            groups[i].add(line);
            groupFound = true;
        }
    }

    if (!groupFound)
    {
        groups.push_back(LineGroup(line));
    }
}

void LineDetection::filterByProximity(const LineGroup& group, 
                                      std::vector<LineGroup>& filtered)
{
    std::vector<const Line*> lines = group.getLines();
    std::vector<Vector> centroids(lines.size());
    for (int i = 0; i < lines.size(); ++i)
    {
        centroids[i] = lines[i]->getCentroid();
    }

    std::vector<int> clusterMemberships;
    DBSCAN::DBSCAN(centroids, 50, 1, clusterMemberships);

    parseClusterMemberships(clusterMemberships, lines, filtered);
}

void LineDetection::parseClusterMemberships(const std::vector<int>& clusterMemberships, 
                                            const std::vector<const Line*>& cluster,
                                                  std::vector<LineGroup>& filtered)
{
    if (cluster.size() != clusterMemberships.size()) return;

    std::unordered_map<int, LineGroup> groups;

    std::unordered_map<int, LineGroup>::iterator it = groups.end();
    for (int i = 0; i < clusterMemberships.size(); ++i) 
    {
        int groupId = clusterMemberships[i];
        it = groups.find(groupId);
        if (it != groups.end()) 
        {
            it->second.add(*cluster[i]);
        } 
        else 
        {
            groups.insert({ groupId, LineGroup(*cluster[i]) });
        }
    }

    for (it = groups.begin(); it != groups.end(); it++) 
    {
        filtered.push_back(it->second);
    }
}

}