#ifndef DETECTION_AND_TRACKING_GREENCOLOR_H
#define DETECTION_AND_TRACKING_GREENCOLOR_H

#include "Color.h"

class GreenColor : public Color
{
public:
    GreenColor();
    ~GreenColor();

	void createTrackbars(string windowName);
};

#endif //DETECTION_AND_TRACKING_GREENCOLOR_H
