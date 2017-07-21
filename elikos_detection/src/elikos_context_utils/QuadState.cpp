#include "QuadState.h"

QuadState::QuadState(const std::string& cameraFrame)
    : cameraFrame_(cameraFrame)
{
}

bool QuadState::update(ros::Time stamp)
{
    timeStamp_ = stamp;
    bool success = false;
    try {
        //tfListener_.waitForTransform("elikos_arena_origin", "elikos_fcu",  stamp, ros::Duration(1.0));
        //tfListener_.lookupTransform("elikos_arena_origin", "elikos_fcu", stamp, origin2fcu_);

        tfListener_.waitForTransform("elikos_arena_origin", "elikos_real_base_link",  stamp, ros::Duration(1.0));
        tfListener_.lookupTransform("elikos_arena_origin", "elikos_real_base_link", stamp, origin2fcu_);


        if (tfListener_.canTransform("elikos_arena_origin", "elikos_attitude", stamp))
        {
            tfListener_.lookupTransform("elikos_arena_origin", "elikos_attitude", stamp, origin2attitude_);
        }
        // Didn't get estimate from external estimator, publish fcu (for sim).
        else 
        {
            origin2attitude_ = tf::StampedTransform(tf::Transform(origin2fcu_.getRotation(), tf::Vector3()), stamp, "elikos_arena_origin", "elikos_attitude");
        }

        tfListener_.waitForTransform("elikos_fcu", cameraFrame_, stamp, ros::Duration(1.0));
        tfListener_.lookupTransform("elikos_fcu", cameraFrame_, stamp, fcu2camera_);
        

        success = true;
    }
    catch (tf::TransformException e)
    {
        ROS_ERROR(e.what());

    }
    return success;
}


