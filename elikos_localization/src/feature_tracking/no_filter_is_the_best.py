#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Je me suis finalement rendu compte qu'on n'avais PAS besoin de filtre!
HA Ha ha ha   ha     ha    ...
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

###
#
# Classes
#
###

class Twist(object):
    def __init__(self,
                 initial_linear_velocity=np.array([0., 0, 0]),
                 initial_angular_velocity=np.array([0., 0, 0])):
        self.linear = initial_linear_velocity
        self.angular = initial_angular_velocity
        self.last_set_time = 0

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

def compute_projection_tranlation(position, rotation):
    """
    Computes a translation of projection based on the given camera position and rotation
    """
    forward = quaternion.rotate_vectors(rotation, np.array([0,0,1]))
    forward *= -position[2]/forward[2]
    return np.array([forward[0], forward[1], 0])

###
#
# ROS section
#
###

#callbacks
def input_mavros_speed(velocity):
    """
    Ros callback for the mavros message.
    """
    global g_quad_twist, g_lock

    g_lock.acquire()

    g_quad_twist.linear[0] = velocity.twist.twist.linear.x
    g_quad_twist.linear[1] = velocity.twist.twist.linear.y
    g_quad_twist.linear[2] = velocity.twist.twist.linear.z

    g_quad_twist.angular[0] = velocity.twist.twist.angular.x
    g_quad_twist.angular[1] = velocity.twist.twist.angular.y
    g_quad_twist.angular[2] = velocity.twist.twist.angular.z

    g_lock.release()


def input_localization_points(point_array):
    """
    Ros callback for the points sent by the localization.
    """
    global g_lock, g_tf_listener, g_arena_points, g_quad_twist, g_frames

    camera_frame = point_array.header.frame_id

    camera_points = np.empty((len(point_array.poses), 3))
    for i, pose in enumerate(point_array.poses):
        camera_points[i, 0] = -pose.position.y
        camera_points[i, 1] = -pose.position.x
        camera_points[i, 2] = -pose.position.z

    (trans,rot) = g_tf_listener.lookupTransform(g_frames["arena_center_frame_id"], camera_frame, rospy.Time())

    _, _, yaw = tf.transformations.euler_from_quaternion(rot)

    #print camera_points[0][2]
    camera_points += np.array([trans[0], trans[1], trans[2]])
    camera_points = quaternion.rotate_vectors(quaternion.from_euler_angles(0, 0, yaw), camera_points)

    trans_projection = compute_projection_tranlation(np.array(trans), quaternion.quaternion(rot[3], rot[0], rot[1], rot[2]))
    camera_points += trans_projection
    print trans_projection


    tmp_publish(camera_points)

    mask = np.ones(3, dtype=np.bool)
    mask[2] = False

    matched_src, matched_dts = match_points_2d(g_arena_points[:, mask], camera_points[:, mask])

    matched_src -= np.array([[trans[:2]]])
    matched_dts -= np.array([[trans[:2]]])
    
    transform = cv2.estimateRigidTransform(matched_src.astype(np.float32), matched_dts.astype(np.float32), False)

    if transform is None:
        rospy.logwarn("The transformation was null! Skipping message.")
        return

    angle_delta = math.atan2(transform[0,0], transform[1,0])
    if angle_delta > 3 * math.pi / 4:
        angle_delta -= math.pi
    elif angle_delta > math.pi / 4:
        angle_delta -= math.pi / 2
    elif angle_delta < - 3 * math.pi / 4:
        angle_delta +=math.pi
    elif angle_delta < - math.pi / 4:
        angle_delta += math.pi / 2
    
    scale = math.sqrt(transform[0,0]**2 + transform[1,0] ** 2)
    dx = transform[0, 2]
    dy = transform[1, 2]

    
    (trans,rot) = g_tf_listener.lookupTransform(g_frames["arena_center_frame_id"], g_frames["fcu_frame_id"], rospy.Time())
    
    delta_rot = quaternion.from_euler_angles(0, 0, angle_delta)
    rot = quaternion.quaternion(rot[3], rot[0], rot[1], rot[2])

    final_rot = delta_rot * rot

    trans =  (trans[0] - dx, trans[1] - dy, trans[2] - camera_points[0, 2])
    g_tf_broadcaster.sendTransform(
        trans,
        (final_rot.x, final_rot.y, final_rot.z, final_rot.w),
        rospy.Time.now(),
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
    topic_mavros_speed = rospy.get_param("~mavros_speed_topic", "/mavros/global_position/local")
    topic_localization_points = rospy.get_param("~topic_input_points", "/localization/features")

    g_frames["arena_center_frame_id"] = rospy.get_param("~arena_center_frame_id", "elikos_arena_origin")
    g_frames["base_link_frame_id"] = rospy.get_param("~base_link_frame_id", "elikos_base_link")
    g_frames["fcu_frame_id"] = rospy.get_param("~fcu_frame_id", "elikos_fcu")
    g_frames["output_position_fcu"] = rospy.get_param("~output_position_fcu_frame_id", "elikos_vision")

    g_tf_listener = tf.TransformListener()
    g_tf_broadcaster = tf.TransformBroadcaster()

    #wait for tf
    g_tf_listener.waitForTransform(g_frames["arena_center_frame_id"], g_frames["base_link_frame_id"], rospy.Time(), rospy.Duration(1))

    rospy.Subscriber(topic_mavros_speed, Odometry, callback=input_mavros_speed)
    rospy.Subscriber(topic_localization_points, PoseArray, callback=input_localization_points)


    g_pub_dbg = rospy.Publisher("/localization/features_debug", PoseArray, queue_size=10)


###
#
# Globals
#
###
g_lock = threading.Lock()
g_tf_listener = None
g_tf_broadcaster = None

g_quad_twist = Twist()
g_arena_points = create_grid_mesh(side_points_number=21, side_mesure=20) - np.array([10, 10, 0])

g_frames = {}

if __name__ == '__main__':
    init_node()
    rospy.spin()




