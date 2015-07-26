//
// Created by andre on 22/07/15.
//

#ifndef ELIKOS_ROS_HEADER_FILE_H
#define ELIKOS_ROS_HEADER_FILE_H

#include "VoCtrl.h"

namespace elikos{
    class SvoCtrl : public VoCtrl{
        void init();
        void reset();
    };
}

#endif //ELIKOS_ROS_HEADER_FILE_H
