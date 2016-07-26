#ifndef MESSAGE_HANDLER_moveit
#define MESSAGE_HANDLER_moveit


#include "moveit_move_group.h"

class MessageHandler_moveit
{
public:
    MessageHandler_moveit();
    ~MessageHandler_moveit();
    void dispatchMessageTarget(const geometry_msgs::PoseStamped::ConstPtr &input);
    geometry_msgs::PoseStamped getTarget();
    bool hasTarget();
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    geometry_msgs::PoseStamped inputTarget_;
    bool hasTarget_;
};

#endif /// MESSAGE_HANDLER_TTF
