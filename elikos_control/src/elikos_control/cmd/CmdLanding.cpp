#include "CmdLanding.h"

CmdLanding::CmdLanding(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)    
{
    cmdPriority_ = PriorityLevel::LANDING;
    cmdCode_ = CmdCode::LAND;

    landingClient_ = nh_->serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    landingCmd_.request.min_pitch = 0.0;
    landingCmd_.request.yaw = 0.0;
    landingCmd_.request.latitude = 0.0;
    landingCmd_.request.longitude = 0.0;
    landingCmd_.request.altitude = 0.0;
    
    targetPosition_.setData(tf::Transform(tf::Quaternion{ 0.0, 0.0, 0.0, 1.0 }, tf::Vector3{ 0.0, 0.0, 0.0 }));
    targetPosition_.child_frame_id_ = SETPOINT;
    targetPosition_.frame_id_ = WORLD_FRAME;

	threshold_ = 0.8;
	nh_->getParam("/elikos_decisionmaking/has_reach_destination_threshold", threshold_);
}

CmdLanding::~CmdLanding()
{
    int i = 0;
}

void CmdLanding::execute()
{
    std_msgs::String msg;
    msg.data = "Landing";
    statePubCommand_.publish(msg);

    ros::Rate rate(30.0);
    // TODO: Essayer a nouveau si le lookup echoue.
    tf::StampedTransform currentPosition;
    try
    {
        tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), currentPosition);
    }
    catch(tf::TransformException e)
    {
        ROS_ERROR("%s",e.what());
    }

    tf::Vector3 groundPosition = currentPosition.getOrigin();
    groundPosition.setZ( 0.0 );
    targetPosition_.setOrigin(groundPosition);

    bool isDone = false;
    while (!isDone)
    {
        ros::spinOnce();
        try 
        {
            tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), lastPosition_);
        }

        catch (tf::TransformException e)
        {
            ROS_ERROR("%s",e.what());
        }

        double distance = lastPosition_.getOrigin().distance(targetPosition_.getOrigin());
        if (distance > threshold_)
        {
            targetPosition_.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(targetPosition_);
            rate.sleep();
        }

        else 
        {
            isDone = true;
            // Atterrir
            landingClient_.call(landingCmd_);
            while(!landingCmd_.response.success)
            {
                 landingClient_.call(landingCmd_);
                 // TODO éventuellement faire bipper le pixhawk pour avertir qu'il souhaite se poser.
            }
        }
    }
    ROS_ERROR("Finished landing command");
}

void CmdLanding::abort()
{
    ///
}
