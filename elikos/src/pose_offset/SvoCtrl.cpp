//
// Created by andre on 22/07/15.
//

#include <pose_offset/SvoCtrl.h>

namespace elikos {
    SvoCtrl::SvoCtrl(ros::NodeHandle nh){
        _nh = nh;
        _ctrl_pub = nh.advertise<std::string>("vo_remote_key", 1);
    }

    void SvoCtrl::init(){
        _ctrl_pub.publish("s");
    }

    void SvoCtrl::reset() {
        _ctrl_pub.publish("r");
    }
}