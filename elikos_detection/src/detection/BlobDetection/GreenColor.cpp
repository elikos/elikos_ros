#include "GreenColor.h"

GreenColor::GreenColor()
{
    H_MIN =  new int(39);
    H_MAX =  new int(88);
    S_MIN =  new int(54);
    S_MAX =  new int(256);
    V_MIN =  new int(30);
    V_MAX =  new int(256);

    PRE_EROSIONS =  new int(3);
    DILATIONS =  new int(20);
    POST_EROSIONS =  new int(0);
}

GreenColor::~GreenColor() {
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

void on_trackbar_G(int, void *) {
	//This function gets called whenever a
    // trackbar position is changed
}

void GreenColor::createTrackbars(string windowName){
    createTrackbar("H_MIN G", windowName, H_MIN, 256, on_trackbar_G);
    createTrackbar("H_MAX G", windowName, H_MAX, 256, on_trackbar_G);
    createTrackbar("S_MIN G", windowName, S_MIN, 256, on_trackbar_G);
    createTrackbar("S_MAX G", windowName, S_MAX, 256, on_trackbar_G);
    createTrackbar("V_MIN G", windowName, V_MIN, 256, on_trackbar_G);
    createTrackbar("V_MAX G", windowName, V_MAX, 256, on_trackbar_G);
    createTrackbar("PRE_EROSIONS G", windowName, PRE_EROSIONS, 256, on_trackbar_G);
    createTrackbar("DILATIONS G", windowName, DILATIONS, 256, on_trackbar_G);
    createTrackbar("POST_EROSIONS G", windowName, POST_EROSIONS, 256, on_trackbar_G);
}
