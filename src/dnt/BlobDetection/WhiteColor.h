#ifndef DETECTION_AND_TRACKING_WHITECOLOR_H
#define DETECTION_AND_TRACKING_WHITECOLOR_H

#include "Color.h"

class WhiteColor : public Color
{
public:
    WhiteColor();
    ~WhiteColor();

	void createTrackbars(string windowName);
};


#endif //DETECTION_AND_TRACKING_WHITECOLOR_H
