//
// Created by andre on 22/07/15.
//

#ifndef ELIKOS_ROS_HEADER_FILE_H
#define ELIKOS_ROS_HEADER_FILE_H

#include <ros/ros.h>
#include "VoCtrl.h"

namespace elikos{
    class SvoCtrl : public VoCtrl{
    public:
        SvoCtrl(ros::NodeHandle nh);
        void init();
        void reset();

    private:
        ros::NodeHandle _nh;
        ros::Publisher _ctrl_pub;
    };
}

#endif //ELIKOS_ROS_HEADER_FILE_H
