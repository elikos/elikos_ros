#ifndef GRID_FITTING_H
#define GRID_FITTING_H

#include "Grid.h"

namespace localization
{

class GridFitting
{
public:
    GridFitting() = default;
    ~GridFitting() = default;

    bool findBestGridModel(const std::vector<Eigen::Vector2f>& intersections, Grid& grid);

private:
    int debug = 0;
};

}

#endif // GRID_FITTING_H