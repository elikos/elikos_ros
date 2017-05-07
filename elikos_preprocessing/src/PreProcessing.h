#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>

namespace preprocessing
{

class PreProcessing
{
public:
    PreProcessing();
    ~PreProcessing() = default;

    void preProcessImage(const cv::Mat& raw, cv::Mat& preProcessed, cv::Mat& bwPreProcessed) const;
    void showCalibTrackBars();

    Eigen::Vector2f translate(const Eigen::Vector2f& v, const Eigen::Vector2f& translation) const;
    Eigen::Vector2f rotate(const Eigen::Vector2f& v, double theta) const;

    inline void setRollPitch(double roll, double pitch);

private:
    double blurSigma = 0.0;

    cv::Mat distortionMap1_;
    cv::Mat distortionMap2_;

    double roll_ = 0.0;
    double pitch_ = 0.0;
};

inline void PreProcessing::setRollPitch(double roll, double pitch)
{
    roll_ = roll;
    pitch_ = pitch; 
}

}

#endif // PRE_PROCESSING_H