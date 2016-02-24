#include "BlobDetection.h"


string BlobDetection::intToString(int number) {
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void on_trackbar(int, void *) {//This function gets called whenever a
    // trackbar position is changed
}

void BlobDetection::createTrackbars() {
    //create window for trackbars
    namedWindow(trackbarWindowName, 0);
    namedWindow(HSVTrackbars, 0);
    namedWindow(shapeDetectTrackbars, 0);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar("PRE BLUR", trackbarWindowName, &PRE_BLUR, 50, on_trackbar);
    createTrackbar("PRE EROSIONS W", trackbarWindowName, &PRE_EROSIONS_W, 256, on_trackbar);
    createTrackbar("DILATIONS_W W", trackbarWindowName, &DILATIONS_W, 256, on_trackbar);
    createTrackbar("POST EROSIONS W", trackbarWindowName, &POST_EROSIONS_W, 50, on_trackbar);
    createTrackbar("PRE EROSIONS G", trackbarWindowName, &PRE_EROSIONS_G, 256, on_trackbar);
    createTrackbar("DILATIONS_W G", trackbarWindowName, &DILATIONS_G, 256, on_trackbar);
    createTrackbar("POST EROSIONS G", trackbarWindowName, &POST_EROSIONS_G, 50, on_trackbar);
    createTrackbar("H_MIN W", HSVTrackbars, &H_MIN_W, 256, on_trackbar);
    createTrackbar("H_MAX W", HSVTrackbars, &H_MAX_W, 256, on_trackbar);
    createTrackbar("S_MIN W", HSVTrackbars, &S_MIN_W, 256, on_trackbar);
    createTrackbar("S_MAX W", HSVTrackbars, &S_MAX_W, 256, on_trackbar);
    createTrackbar("V_MIN W", HSVTrackbars, &V_MIN_W, 256, on_trackbar);
    createTrackbar("V_MAX W", HSVTrackbars, &V_MAX_W, 256, on_trackbar);
    createTrackbar("H_MIN G", HSVTrackbars, &H_MIN_G, 256, on_trackbar);
    createTrackbar("H_MAX G", HSVTrackbars, &H_MAX_G, 256, on_trackbar);
    createTrackbar("S_MIN G", HSVTrackbars, &S_MIN_G, 256, on_trackbar);
    createTrackbar("S_MAX G", HSVTrackbars, &S_MAX_G, 256, on_trackbar);
    createTrackbar("V_MIN G", HSVTrackbars, &V_MIN_G, 256, on_trackbar);
    createTrackbar("V_MAX G", HSVTrackbars, &V_MAX_G, 256, on_trackbar);
    createTrackbar("H_MIN R", HSVTrackbars, &H_MIN_R, 256, on_trackbar);
    createTrackbar("H_MAX R", HSVTrackbars, &H_MAX_R, 256, on_trackbar);
    createTrackbar("S_MIN R", HSVTrackbars, &S_MIN_R, 256, on_trackbar);
    createTrackbar("S_MAX R", HSVTrackbars, &S_MAX_R, 256, on_trackbar);
    createTrackbar("V_MIN R", HSVTrackbars, &V_MIN_R, 256, on_trackbar);
    createTrackbar("V_MAX R", HSVTrackbars, &V_MAX_R, 256, on_trackbar);

    // Create shape detector trackbars
    createTrackbar("Canny Thresh 1", shapeDetectTrackbars, &CANNY_THRESH1, 256, on_trackbar);
    createTrackbar("Canny Thresh 2", shapeDetectTrackbars, &CANNY_THRESH2, 256, on_trackbar);
    createTrackbar("Canny Aperture", shapeDetectTrackbars, &CANNY_APERTURE, 5, on_trackbar);
    createTrackbar("Poly Area Min", shapeDetectTrackbars, &POLY_AREA_MIN, 50000, on_trackbar);
    createTrackbar("Poly Area Max", shapeDetectTrackbars, &POLY_AREA_MAX, 50000, on_trackbar);
    createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat",
                   shapeDetectTrackbars, &MORPH_OP, 4, on_trackbar);
    createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", shapeDetectTrackbars, &MORPH_ELEMENT, 2,
                    on_trackbar);
    createTrackbar("Kernel size:\n 2n +1", shapeDetectTrackbars, &MORPH_SIZE, 21, on_trackbar);
    createTrackbar("Max distance", shapeDetectTrackbars, &MAX_DIST, 50000, on_trackbar);

}



void BlobDetection::trackBlobs(const cv::Mat &input, cv::Mat &output_w, cv::Mat &output_r, cv::Mat &output_g) {
    currentImage = input; //AJOUT

    vector<Mat> thresholds;
    vector<Color*> colors = {new WhiteColor, new GreenColor, new RedColor};

    for(Color* color : colors) {
        thresholds.push_back(color->generateThreshold(currentImage));
        color->trackFilteredObjects(currentImage);
        color->printFoundObjects();
        color->drawFoundObjects(currentImage);
        delete color;
    }




    output_w = thresholds[WHITE]; // AJOUT
    output_g = thresholds[GREEN]; // AJOUT
    output_r = thresholds[RED]; // AJOUT
}


