#include "CmdTakeOff.h"

CmdTakeOff::CmdTakeOff(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    cmdPriority_ = PriorityLevel::TAKEOFF_PRIORITY;
    cmdCode_ = CmdCode::TAKEOFF;

    stateSub_ = nh_->subscribe<mavros_msgs::State>("mavros/state", 10, &CmdTakeOff::stateCallBack, this);
    armingClient_ = nh_->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    setModeClient_ = nh_->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    offbSetMode_.request.custom_mode = "OffBoard";
    armCmd_.request.value = true;

    double takeoff_altitude = 1;
    nh_->getParam("/elikos_decisionmaking/takeoff_altitude", takeoff_altitude);
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, takeoff_altitude }));
    targetPosition_.child_frame_id_ = SETPOINT;
    targetPosition_.frame_id_ = WORLD_FRAME;

	threshold_ = 0.8;
	nh_->getParam("/elikos_decisionmaking/has_reach_destination_threshold", threshold_);
}

CmdTakeOff::~CmdTakeOff()
{
    int i = 0;
}

void CmdTakeOff::stateCallBack(const mavros_msgs::State::ConstPtr& msg)
{
    currentState_ = *msg;
}

void CmdTakeOff::execute()
{
    std_msgs::String msg;
    msg.data = "Take off";
    statePubCommand_.publish(msg);

    ros::Rate rate(5.0);


    bool initialPositionFound = false;
    while(!initialPositionFound)
    {
        try {
            tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
            initialPositionFound = true;
        } catch (tf::TransformException e) {
            ROS_ERROR("%s",e.what());
        }
    }

    targetPosition_.getOrigin().setX(lastPosition_.getOrigin().x());
    targetPosition_.getOrigin().setY(lastPosition_.getOrigin().y());

    //send a few setpoints before starting
    for(int i = 0; ros::ok() && i < 10; ++i)
    {
        targetPosition_.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(targetPosition_);
        ros::spinOnce();
        rate.sleep();
    }

    bool isDone = false;
    while (!isDone)
    {
       
        lastRequest_ = ros::Time::now();

        try {
            tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
        } catch (tf::TransformException e) {
            ROS_ERROR("Last position : %s",e.what());
        }

        double distance = lastPosition_.getOrigin().distance(targetPosition_.getOrigin());
        if (distance > threshold_)
        {
            targetPosition_.getOrigin().setX(lastPosition_.getOrigin().x());
            targetPosition_.getOrigin().setY(lastPosition_.getOrigin().y());
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
        } 
        else 
        {
            isDone = true;
        }
         ros::spinOnce();
         rate.sleep();
    }
}


void CmdTakeOff::abort()
{
    ///
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

