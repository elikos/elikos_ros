#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Fallback du merge des points.
"""
import threading
import math

import numpy as np
import quaternion

import cv2

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
import elikos_ros.msg as elikos_ros

import message_interface as msgs


###
#
# Math methods
#
###
def create_grid_mesh(side_points_number, side_mesure):
    u"""
    Creates a grid mesh containing side_points_number² points ane mesuring side_mesure x side_mesure
    """
    x = np.linspace(0, side_mesure, num=side_points_number)
    y = np.linspace(0, side_mesure, num=side_points_number)
    z = np.array([0])
    xs, ys, zs = np.meshgrid(x, y, z)
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)

def closest_point(point_array, position):
    """
    @param node_array A numpy array of positions size:(x, 3)
    @param position A numpy vector size:(3,)
    @return the index of the closest node in the array of input nodes
    @rtype ndarray(int)
    """
    if point_array.shape[0] is 0:
        return None
    deltas = point_array - position
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

def match_points_2d(src, dst):
    matched_estimation = np.empty((1, 0, 2))
    matched_mesurement = np.empty((1, 0, 2))
    for i in xrange(np.size(dst, axis=0)):
        position = dst[i,:]
        closest_point_index = closest_point(src, position)
        matched_estimation = np.append(matched_estimation, np.array([[src[closest_point_index]]]), axis=1)
        matched_mesurement = np.append(matched_mesurement, np.array([[position]]), axis=1)
    return matched_estimation, matched_mesurement


###
#
# ROS section
#
###

#callbacks
def input_localization_points_point_array(point_array):
    """
    Ros callback for the points sent by the localization.
    """
    global g_tf_listener

    camera_frame = point_array.header.frame_id

    camera_points = np.empty((len(point_array.poses), 3))
    for i, pose in enumerate(point_array.poses):
        camera_points[i, 0] = pose.position.x
        camera_points[i, 1] = pose.position.y
        camera_points[i, 2] = pose.position.z

    localize_drone(camera_points, camera_frame, point_array.header.stamp)


def input_localization_points(localization_msg):
    #type: (elikos_ros.IntersectionArray)->None

    points_image, points_arena = msgs.deserialize_intersections(localization_msg)
    stamp = localization_msg.header.stamp
    frame = localization_msg.header.frame_id

    localize_drone(points_arena, frame, stamp)


def localize_drone(input_points_3d, input_points_frame, frame_time):
    # type: (np.ndarray, str, rospy.Time)->None

    global g_tf_listener, g_frames, g_tf_broadcaster

    g_tf_listener.waitForTransform(input_points_frame, g_frames["arena_center_frame_id"], frame_time, rospy.Duration(3.0))
    (trans, rot) = g_tf_listener.lookupTransform(input_points_frame, g_frames["arena_center_frame_id"], frame_time)

    input_points_3d -= trans
    input_points_3d = quaternion.rotate_vectors(quaternion.quaternion(rot[3], rot[0], rot[1], rot[2]), input_points_3d)

    mask = np.ones(3, dtype=np.bool)
    mask[2] = False

    matched_src, matched_dts = match_points_2d(g_arena_points[:, mask], input_points_3d[:, mask])

    transform = cv2.estimateRigidTransform(matched_src.astype(np.float32), matched_dts.astype(np.float32), False)

    if transform is None:
        rospy.logwarn("The transformation was null! Skipping message.")
        return

    angle_delta = math.atan2(transform[0, 0], transform[1, 0])
    if angle_delta > 3 * math.pi / 4:
        angle_delta -= math.pi
    elif angle_delta > math.pi / 4:
        angle_delta -= math.pi / 2
    elif angle_delta < - 3 * math.pi / 4:
        angle_delta += math.pi
    elif angle_delta < - math.pi / 4:
        angle_delta += math.pi / 2

    scale = math.sqrt(transform[0, 0] ** 2 + transform[1, 0] ** 2)
    dx = transform[0, 2]
    dy = transform[1, 2]

    tmp_publish(input_points_3d)

    (trans, rot) = g_tf_listener.lookupTransform(g_frames["arena_center_frame_id"], g_frames["fcu_frame_id"],
                                                 rospy.Time())

    delta_rot = quaternion.from_euler_angles(0, 0, angle_delta)
    rot = quaternion.quaternion(rot[3], rot[0], rot[1], rot[2])

    final_rot = delta_rot * rot

    trans = (trans[0] - dx, trans[1] - dy, trans[2] + input_points_3d[0, 2])
    g_tf_broadcaster.sendTransform(
        trans,
        (final_rot.x, final_rot.y, final_rot.z, final_rot.w),
        rospy.Time.frame_time(),
        g_frames["output_position_fcu"],
        g_frames["arena_center_frame_id"]
    )


def tmp_publish(camera_points):
    global g_pub_dbg, g_frames, g_arena_points
    output_message = PoseArray()

    for i in xrange(np.size(camera_points, axis=0)):
        p = Pose()
        p.position.x = camera_points[i, 0]
        p.position.y = camera_points[i, 1]
        p.position.z = camera_points[i, 2]

        output_message.poses.append(p)
    for i in xrange(np.size(g_arena_points, axis=0)):
        p = Pose()
        p.position.x = g_arena_points[i, 0]
        p.position.y = g_arena_points[i, 1]
        p.position.z = g_arena_points[i, 2]

        output_message.poses.append(p)

    output_message.header = Header()
    output_message.header.stamp = rospy.Time.now()
    output_message.header.frame_id = g_frames["arena_center_frame_id"]

    g_pub_dbg.publish(output_message)


def init_node():
    """
    Initialises the node.
    """
    global g_tf_listener, g_frames, g_pub_dbg, g_tf_broadcaster
    rospy.init_node("feature_tracking")

    #speed is from mavros vecause TF dosen't store that
    topic_mavros_speed = rospy.get_param("mavros_speed_topic", "/mavros/global_position/local")
    topic_localization_points = rospy.get_param("topic_localization_points", "/localization/features")
    topic_publication_pose = rospy.get_param("topic_publication_pose", "/localization/drone_pose")

    g_frames["arena_center_frame_id"] = rospy.get_param("arena_center_frame_id", "elikos_arena_origin")
    g_frames["base_link_frame_id"] = rospy.get_param("arena_center_frame_id", "elikos_base_link")
    g_frames["fcu_frame_id"] = rospy.get_param("arena_center_frame_id", "elikos_fcu")
    g_frames["output_position_fcu"] = rospy.get_param("output_position_fcu_frame_id", "elikos_vision")

    g_tf_listener = tf.TransformListener()
    g_tf_broadcaster = tf.TransformBroadcaster()

    #wait for tf
    g_tf_listener.waitForTransform(g_frames["arena_center_frame_id"], g_frames["base_link_frame_id"], rospy.Time(), rospy.Duration(1))

    rospy.Subscriber(topic_localization_points, elikos_ros.IntersectionArray, callback=input_localization_points)


    g_pub_dbg = rospy.Publisher("/localization/features_debug", PoseArray, queue_size=10)


###
#
# Globals
#
###
g_tf_listener = None
g_tf_broadcaster = None

g_arena_points = create_grid_mesh(side_points_number=21, side_mesure=20) - np.array([10, 10, 0])

g_frames = {}

if __name__ == '__main__':
    init_node()
    rospy.spin()




