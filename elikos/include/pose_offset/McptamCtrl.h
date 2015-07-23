//
// Created by andre on 22/07/15.
//

#include "VoCtrl.h"

#ifndef ELIKOS_ROS_MCPTAM_CTRL_H
#define ELIKOS_ROS_MCPTAM_CTRL_H

namespace elikos{
    class McptamCtrl : VoCtrl {
        void init();
        void reset();
    };
}

#endif //ELIKOS_ROS_MCPTAM_CTRL_H
