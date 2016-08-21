#include "WhiteColor.h"

WhiteColor::WhiteColor()
{
    H_MIN = new int(0);
    H_MAX = new int(52);
    S_MIN = new int(0);
    S_MAX = new int(149);
    V_MIN = new int(140);
    V_MAX = new int(256);

    PRE_EROSIONS =  new int(12);
    DILATIONS =  new int(25);
    POST_EROSIONS =  new int(5);
}

WhiteColor::~WhiteColor() {
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

void on_trackbar_W(int, void *) {
	//This function gets called whenever a
    // trackbar position is changed
}

void WhiteColor::createTrackbars(string windowName){
    createTrackbar("H_MIN W", windowName, H_MIN, 256, on_trackbar_W);
    createTrackbar("H_MAX W", windowName, H_MAX, 256, on_trackbar_W);
    createTrackbar("S_MIN W", windowName, S_MIN, 256, on_trackbar_W);
    createTrackbar("S_MAX W", windowName, S_MAX, 256, on_trackbar_W);
    createTrackbar("V_MIN W", windowName, V_MIN, 256, on_trackbar_W);
    createTrackbar("V_MAX W", windowName, V_MAX, 256, on_trackbar_W);
    createTrackbar("PRE_EROSIONS W", windowName, PRE_EROSIONS, 256, on_trackbar_W);
    createTrackbar("DILATIONS W", windowName, DILATIONS, 256, on_trackbar_W);
    createTrackbar("POST_EROSIONS W", windowName, POST_EROSIONS, 256, on_trackbar_W);
}
