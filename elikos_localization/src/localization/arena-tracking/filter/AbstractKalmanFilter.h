#include "KalmanFilter.h"

#include <Eigen/Core>

template <std::size_t Nx, std::size_t Nz, std::size_t Nu = 1>
class AbstractKalmanFilter : public KalmanFilter<Nx, Nz, Nu>
{
public:

    AbstractKalmanFilter() = default;
    ~AbstractKalmanFilter() = default;

    // State vector estimate
    Eigen::Matrix<float, 1, Nx> x;

    // State transition function (Transform from estimate to prediction)
    Eigen::Matrix<float, Nx, Nx> F;

    // State covariance matrix
    Eigen::Matrix<float, Nx, Nx> P;

    // Control input function (Convert inputs to state space)
    Eigen::Matrix<float, Nx, Nu> B; 

    // Process covariance matrix (Noise added by process model)
    Eigen::Matrix<float, Nx, Nx> Q;

    // Measurement function (Transform from state space to measurement space)
    Eigen::Matrix<float, Nz, Nx> H;

    // Measurement covariance matrix (Noise added by measurements)
    Eigen::Matrix<float, Nz, Nz> R;

    virtual void predict(const Eigen::Matrix<float, 1, Nu>& u = Eigen::Matrix<float, 1, Nu>());

    virtual void update(const Eigen::Matrix<float, 1, Nz>& z);
};

template <std::size_t Nx, std::size_t Nz, std::size_t Nu>
void AbstractKalmanFilter<Nx, Nz, Nu>::predict(const Eigen::Matrix<float, 1, Nu>& u)
{
    x = F * x + B * u;
    P = F * P * F.transpose() + Q;
}

template <std::size_t Nx, std::size_t Nz, std::size_t Nu>
void AbstractKalmanFilter<Nx, Nz, Nu>::update(const Eigen::Matrix<float, 1, Nz>& z)
{
    Eigen::Matrix<float, 1, Nz> y = z - H * x;
    Eigen::Matrix<float, 1, Nx> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    Eigen::Matrix<float, Nx, Nx> I = Eigen::Matrix<float, Nx, Nx>::Identity();

    x = x + K * y;
    P = (I - K * H) * P;
}