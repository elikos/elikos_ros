# pose_offset node documentation
The point of this node is to allow for visual odometry (VO) resets while in flight. Since VO
methods start over at 0,0,0 when reinitialized. It's important to offset the new pose estimate
with the current position estimated by the fcu, or else the vehicle will react violently with
potential mismatch between vision and current position estimates.

## Parameters
* publish_covariance(bool, true)
  * Choose between publishing `PoseStamped` and `PoseWithCovarianceStamped`
* receive_pose_with_covariance
  * Choose between receiving `PoseStamped` and `PoseWithCovarianceStamped`
* rate(int, 100)
  * Rate at which this node will run
* vo_method(string, "svo")
  * What VO method is being used. Supported methods: "svo", "mcptam"

## Subscribed topics
* ~/local_position(geometry_msgs/PoseStamped)
  * Pose of the vehicle as estimated by fcu
* ~/pose_in(geometry_msgs/PoseStamped or geometry_msgs/PoseWithCovarianceStamped)
  * Pose of the vehicle as estimated by vision (VO) node

## Published topics
* ~/pose_out(geometry_msgs/PoseStamped or geometry_msgs/PoseWithCovarianceStamped)
  * Vision pose with the correct offset addded to it


