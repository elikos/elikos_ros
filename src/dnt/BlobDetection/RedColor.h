#ifndef DETECTION_AND_TRACKING_REDCOLOR_H
#define DETECTION_AND_TRACKING_REDCOLOR_H

#include "Color.h"

class RedColor : public Color
{
public:
    RedColor();
    ~RedColor();
    
	void createTrackbars(string windowName);
};


#endif //DETECTION_AND_TRACKING_REDCOLOR_H
