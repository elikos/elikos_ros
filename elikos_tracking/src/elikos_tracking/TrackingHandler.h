#include <vector>
#include "robot.h"
#include <elikos_ros/RobotRaw.h>
#include <elikos_ros/RobotRawArray.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>  //for using the function sleep
#include <unordered_map>
#include <memory>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

const int NUM_ROBOTS_PER_COLOR = 5;


class TrackingHandler {
    public:

    static TrackingHandler* getInstance();

    std::shared_ptr<Robot> getRobotAtIndex(int index);
    void clearRobots();
    int DoMatch(geometry_msgs::Point inputPoint, uint8_t color);
    static void subCallback(const elikos_ros::RobotRawArray::ConstPtr& msg);

   private:

    TrackingHandler();
    static TrackingHandler* handlerInstance_;
    //std::vector<std::unique_ptr<Robot>> redRobots_;
    //std::vector<std::unique_ptr<Robot>> greenRobots_;
    std::vector<std::shared_ptr<Robot>> robotsVec_;

};
