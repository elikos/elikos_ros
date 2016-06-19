#include "RedColor.h"

RedColor::RedColor()
{
    H_MIN =  new int(0);
    H_MAX =  new int(48);
    S_MIN =  new int(93);
    S_MAX =  new int(256);
    V_MIN =  new int(141);
    V_MAX =  new int(256);

    PRE_EROSIONS =  new int(12);
    DILATIONS =  new int(25);
    POST_EROSIONS =  new int(5);
}

RedColor::~RedColor() {
    delete H_MIN;
    delete H_MAX;
    delete S_MIN;
    delete S_MAX;
    delete V_MIN;
    delete V_MAX;
    delete PRE_EROSIONS;
    delete DILATIONS;
    delete POST_EROSIONS;
 }

void on_trackbar_R(int, void *) {
	//This function gets called whenever a
    // trackbar position is changed
}

void RedColor::createTrackbars(string windowName){
    createTrackbar("H_MIN R", windowName, H_MIN, 256, on_trackbar_R);
    createTrackbar("H_MAX R", windowName, H_MAX, 256, on_trackbar_R);
    createTrackbar("S_MIN R", windowName, S_MIN, 256, on_trackbar_R);
    createTrackbar("S_MAX R", windowName, S_MAX, 256, on_trackbar_R);
    createTrackbar("V_MIN R", windowName, V_MIN, 256, on_trackbar_R);
    createTrackbar("V_MAX R", windowName, V_MAX, 256, on_trackbar_R);
    createTrackbar("PRE_EROSIONS R", windowName, PRE_EROSIONS, 256, on_trackbar_R);
    createTrackbar("DILATIONS R", windowName, DILATIONS, 256, on_trackbar_R);
    createTrackbar("POST_EROSIONS R", windowName, POST_EROSIONS, 256, on_trackbar_R);
}
