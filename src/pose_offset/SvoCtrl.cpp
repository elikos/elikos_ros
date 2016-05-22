//
// Created by andre on 22/07/15.
//

#include <pose_offset/SvoCtrl.h>
#include <std_msgs/String.h>

namespace elikos {
    SvoCtrl::SvoCtrl(ros::NodeHandle nh){
        _nh = nh;
        _ctrl_pub = nh.advertise<std_msgs::String>("vo_remote_key", 1);
    }

    void SvoCtrl::init(){
        std_msgs::String msg;
        msg.data = "s";
        _ctrl_pub.publish(msg);
    }

    void SvoCtrl::reset() {
        std_msgs::String msg;
        msg.data = "r";
        _ctrl_pub.publish(msg);
    }
}