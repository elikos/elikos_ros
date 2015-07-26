//
// Created by andre on 22/07/15.
//

#include <ros/ros.h>
#include <pose_offset/McptamCtrl.h>
#include <mcptam_msgs/Reset.h>
#include <std_srvs/Empty.h>

namespace elikos {
    McptamCtrl::McptamCtrl(ros::NodeHandle nh){
        this->_resetClient = nh.serviceClient<mcptam_msgs::Reset>("mcptam/reset");
        this->_initClient = nh.serviceClient<std_srvs::Empty>("mcptam/init");

        if(!this->_resetClient.waitForExistence(ros::Duration(10,0)) |
           !this->_initClient.waitForExistence(ros::Duration(10,0))){
            //caca
        }
    }

    void McptamCtrl::init() {
        std_srvs::Empty e;
        this->_initClient.call(e);
    }

    void McptamCtrl::reset() {
        mcptam_msgs::Reset r;
        r.request.bReInit = true;
        r.request.bSavePose = false;
        this->_resetClient.call(r);
    }
}
