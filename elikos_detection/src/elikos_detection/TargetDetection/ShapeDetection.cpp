#include "ShapeDetection.h"

ShapeDetection::ShapeDetection() {}

void ShapeDetection::detect(const cv::Mat& input, cv::Mat& output_w,
                            cv::Mat& output_r, cv::Mat& output_g,
                            cv::Mat& output,
                            std::vector<RobotDesc>& outputRobotsArray,
                            BlobDetection& blobDetection) {
    // TODO:
    // - Get the detected shape (x,y) and compare them with the blob detected
    // (x,y)
    // - Find a better way to give IDs to targets
    // - Filter the white spots in the image (make green and red bigger?)
    //      -- Use some filter (Median, Gaussian, Bilateral, etc.)
    //      -- Make pixels have same colors as pixels around (weights) --> Erode + Dilate + Erode

    output = input;  // Initialize output

    /******************************************************************************
    * Resize Image (for better detection)
    ******************************************************************************/
    cv::Size originalSize = input.size();
    double resizedWidth = 300;
    double resizedHeight =
        (originalSize.height * resizedWidth) / originalSize.width;
    const double widthRatio = originalSize.width / resizedWidth;
    const double heightRatio = originalSize.height / resizedHeight;

    cv::Mat resizedInput, bluredInput, hsvInput, threshInput;
    cv::resize(input, resizedInput, cv::Size(resizedWidth, resizedHeight));

    /******************************************************************************
    * Blur the resized image and get threshold
    ******************************************************************************/

    threshInput = blobDetection.getGreen()->generateThreshold(resizedInput);
    threshInput += blobDetection.getRed()->generateThreshold(resizedInput);

    /******************************************************************************
    *  Find Contours in the threshold
    ******************************************************************************/
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(threshInput, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    /******************************************************************************
    * Detect shapes in the found contours
    ******************************************************************************/
    // cv::RNG rng(12345);
    for (int i = 0; i < contours.size(); i++) {
        // Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
        //                       rng.uniform(0, 255));

        vector<cv::Point> shape = contours[i];
        vector<vector<cv::Point> > tempContainer;  // Needed to drawContours...
        cv::Moments shapeMoments = cv::moments(shape);

        double m00 = shapeMoments.m00;
        if (m00 == 0) m00 = 0.1;

        int cX = (int)(shapeMoments.m10 / m00 * widthRatio);
        int cY = (int)(shapeMoments.m01 / m00 * heightRatio);

        // Get the shape approx vertex
        double perimeter = cv::arcLength(shape, true);
        vector<cv::Point> shapeVertex;
        cv::approxPolyDP(shape, shapeVertex, 0.08 * perimeter, true);

        if (shapeVertex.size() >= 4 &&
            shapeVertex.size() <= 6) {  // It's a ROOMBA !!!
            // Correct the (x,y) position to the real image coords
            for (size_t j = 0; j < shape.size(); ++j) {
                shape[j].x *= widthRatio;
                shape[j].y *= heightRatio;
            }
            tempContainer.push_back(shape);
            cv::drawContours(output, tempContainer, 0, cv::Scalar(0, 255, 0),
                             2);
            cv::putText(output, "ROOMBA", cv::Point(cX, cY),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255),
                        2);
        }

        // cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0,
        //  Point());
    }
}