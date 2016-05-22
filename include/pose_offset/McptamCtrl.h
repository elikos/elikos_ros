//
// Created by andre on 22/07/15.
//

#include <ros/node_handle.h>
#include "VoCtrl.h"

#ifndef ELIKOS_ROS_MCPTAM_CTRL_H
#define ELIKOS_ROS_MCPTAM_CTRL_H

namespace elikos{
    class McptamCtrl : public VoCtrl {
    public:
        McptamCtrl(ros::NodeHandle nh);
        void init();
        void reset();

    private:
        ros::ServiceClient _resetClient;
        ros::ServiceClient _initClient;
    };
}

#endif //ELIKOS_ROS_MCPTAM_CTRL_H
