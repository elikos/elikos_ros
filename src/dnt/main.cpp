#include "circleDetectionTestMain.h"
#include "trackBlobsTestMain.h"
#include "robotsDetectionMain.h"
#include "OpticalFlowMain.h"

int main()
{
    // Décommenter pour tester le main correspondant.
    //circleDetectionTestMain();
    //trackBlobsTestMain();
    //int returnValue = trackRobotsTestMain();
    //int returnValue = opticalFlowMain();
    int returnValue = blobTrackingMain();
    return returnValue;
}
