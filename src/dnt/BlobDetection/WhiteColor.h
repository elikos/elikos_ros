#ifndef DETECTION_AND_TRACKING_WHITECOLOR_H
#define DETECTION_AND_TRACKING_WHITECOLOR_H

#include "Color.h"

class WhiteColor : public Color
{
public:
    WhiteColor();
    ~WhiteColor();

    virtual void printFoundObjects();
    virtual void drawFoundObjects(Mat& image);

};


#endif //DETECTION_AND_TRACKING_WHITECOLOR_H
