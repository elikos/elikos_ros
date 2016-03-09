//
// Created by andre on 22/07/15.
// This file defines an interface to control visual odometry packages
//

#ifndef ELIKOS_ROS_VO_CTRL_H
#define ELIKOS_ROS_VO_CTRL_H

namespace elikos {
    enum SupportedVO { SVO, MCPTAM };

    class VoCtrl{
    public:
        virtual void init() = 0;
        virtual void reset() = 0;


    private:

    };
}

#endif //ELIKOS_ROS_VO_CTRL_H
