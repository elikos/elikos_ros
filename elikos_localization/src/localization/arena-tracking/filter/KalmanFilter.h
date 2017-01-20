#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Core>

template <std::size_t Nx, std::size_t Nz, std::size_t Nu = 1>
class KalmanFilter 
{
public:
    virtual void predict(const Eigen::Matrix<float, 1, Nu>& u = Eigen::Matrix<float, 1, Nu>()) = 0;

    virtual void update(const Eigen::Matrix<float, 1, Nz>& z) = 0;
};


#endif // KALMAN_FILTER_H