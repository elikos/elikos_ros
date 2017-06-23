import rospy
import tf
import position_logger

rospy.init_node("reality_logger", anonymous=True)

braodcaster = tf.TransformBroadcaster()


logger = position_logger.GroundTruth()

def callback():
    global braodcaster, logger
    braodcaster.sendTransform(
        logger.position,
        (logger.orientation.x, logger.orientation.y, logger.orientation.z, logger.orientation.w),
        rospy.Time.now(),
        "elikos_real_base_link",
        "elikos_arena_origin"
    )

logger.callback = callback

rospy.spin()