//
// Created by olivier on 10/2/16.
//

#ifndef LOCALIZATION_IMAGE_PROCESSOR_H
#define LOCALIZATION_IMAGE_PROCESSOR_H

#include <opencv2/core/core.hpp>
#include "Line.h"

namespace localization {

class LineGroup;

class ImageProcessor
{
public:

    static ImageProcessor* getInstance();
    static void freeInstance();

    void processImage(cv::Mat input);

private:
    static ImageProcessor* instance_;
    cv::Mat image_;

    cv::Mat distortionMap1_;
    cv::Mat distortionMap2_;

    std::vector<Line> lineCluster_;

    cv::Mat vLines_, hLines_;

    void preProcess(const cv::Mat& raw, cv::Mat& preProcessed);
    void findEdges(const cv::Mat& src, cv::Mat& edges);
    void findLines(const cv::Mat& edges, cv::Mat& lines);
    void analyzeLineCluster();
    void buildLineArray(const std::vector<cv::Vec2f>& lineCluster);


    void drawRawLines(cv::Mat& dst, const std::vector<cv::Vec2f> &raw_lines) const;
    void drawLineGroup(cv::Mat& dst, const LineGroup& group);

    ImageProcessor();
    ~ImageProcessor() = default;
};

}


#endif // LOCALIZATION_IMAGE_PROCESSOR_H
