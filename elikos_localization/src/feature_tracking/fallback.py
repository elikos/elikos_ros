#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Fallback du merge des points.
"""
import threading
import math
import sys

import numpy as np
import quaternion

import cv2
import opengv

import rospy
import tf
import message_filters

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import elikos_main.msg as elikos_main

import message_interface as msgs
import point_manipulation as pt_manip
import point_matching as pt_match
import message_filters_extras

###
#
# Classes
#
###
class LocalizationUnavailableException(Exception):
    u"""
    Excepition thrown when the drone cannot localize itself.
    """
    def __init__(self, message = "localization was unavailable", cause=None):
        super(LocalizationUnavailableException, self).__init__(message + u', caused by ' + (repr(cause) if cause is not None else ''))
        self.cause = cause


class FullMessage:
    def __init__(self, localization_msg, camera_info):
        self.header = localization_msg.header
        self.localization_msg = localization_msg
        self.camera_info = camera_info


class Configuration:
    def __init__(self):
        self.publish_fcu_on_failure = rospy.get_param(
            "~publish_fcu_on_failure",
            False
        )
        self.topic_localization_points_prefix = rospy.get_param(
            "~topic_localization_points_prefix",
            "localization/features_"
        )
        self.topic_camera_info_prefix = rospy.get_param(
            "~topic_camera_info_prefix",
            "localization/camera_info_"
        )

        self.initial_drone_position = np.array(
            rospy.get_param("~initial_drone_pos", [0, 0, 0])
        )
        self.initial_drone_rotation = quaternion.as_quat_array(
            np.array(
                rospy.get_param("~initial_drone_rot", [1, 0, 0, 0])
            )
        )
        self.stabilization_time = rospy.get_param(
            "~stabilization_time",
            3.0
        )
        self.camera_number = rospy.get_param(
            "~camera_number",
            1
        )
        self.watchdog_max_message_delay = rospy.get_param(
            "~watchdog_max_message_delay",
            1.0
        )

        self.frames = {}
        self.frames["arena_center"] = rospy.get_param("~arena_center_frame_id", "elikos_arena_origin")
        self.frames["fcu"] = rospy.get_param("~fcu_frame_id", "elikos_fcu")
        self.frames["output"] = rospy.get_param("~output_position_fcu_frame_id", "elikos_vision")



class GlobalState:
    def __init__(self):
        self.last_fcu_position = None
        self.configuration = Configuration()

        if self.configuration.camera_number <= 0:
            rospy.logwarn("Not listening on any camera. Have you checked the camera_number parameter?")

        # Creating the message filters listeners.
        self.camera_listeners = []


        for i in xrange(self.configuration.camera_number):
            points_subscriber_name = self.configuration.topic_localization_points_prefix + str(i)
            camera_info_subscriber_name = self.configuration.topic_camera_info_prefix + str(i)

            points_filter_subscriber = message_filters.Subscriber(
                points_subscriber_name,
                elikos_main.IntersectionArray,
                queue_size=1
            )
            camera_info_filter_subscriber = message_filters.Subscriber(
                camera_info_subscriber_name,
                CameraInfo,
                queue_size=8
            )

            message_filter_subscriber = message_filters.TimeSynchronizer(
                [
                    points_filter_subscriber,
                    camera_info_filter_subscriber
                ],
                5
            )

            self.camera_listeners.append(
                message_filters_extras.Combiner(
                    message_filter_subscriber,
                    lambda msg_pts, msg_ci : FullMessage(msg_pts, msg_ci)
                )
            )

        self.synchonyser = message_filters.ApproximateTimeSynchronizer(
            self.camera_listeners,
            1,
            0.2
        )

        self.current_callback = None

        self.total_messages_processed = 0
        self.last_message_time = rospy.Time(0)

    def register_a_processed_message(self):
        u"""
        Call to notify the global state that a messages is being processed, will be soon processed or was just processed.
        :return: None
        """
        self.total_messages_processed += 1
        self.last_message_time = rospy.Time.now()


###
#
# Math methods
#
###
def mean_of_times(times):
    # type: (list[rospy.Time])->rospy.Time
    u"""
    Calculates the mean of times
    :param times: the times (a generator)
    :return: the time that is the mean of the time
    """
    base_time = next(times)
    count = 1
    sum_deltas = rospy.Duration()
    for t in times:
        sum_deltas += (t - base_time)
        count += 1
    return base_time + (sum_deltas / float(count))

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

def yaw_from_quaterion(q):
    return math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)

###
#
# ROS section
#
###


def no_estimate(time, global_state):
    if global_state.configuration.publish_fcu_on_failure:
        publish_fcu_transform(
            global_state,
            global_state.last_fcu_position[0],
            global_state.last_fcu_position[1],
            time)

def input_localization_points(*args):
    #type: (tuple[FullMessage, GlobalState])->None

    global g_arena_points

    #Last argument is the global state
    global_state = args[-1]

    global_state.register_a_processed_message()

    time = mean_of_times(msg.header.stamp for msg in args[:-1])

    #array of 3d points
    camera_infos = []
    intersections_2d = []
    points_3d = []
    list_of_matches = []

    for full_msg in args[:-1]:
        points_image, points_arena = msgs.deserialize_intersections(full_msg.localization_msg)

        msg_time = full_msg.localization_msg.header.stamp
        msg_frame = full_msg.localization_msg.header.frame_id

        try:
            transformed_points_arena = transform_points(
                points_arena,
                msg_frame,
                global_state.configuration.frames["arena_center"],
                msg_time
            )

            points_3d.append(transformed_points_arena)

            camera_infos.append(full_msg.camera_info)
            intersections_2d.append(points_image)

            list_of_matches.append(pt_match.match_points(transformed_points_arena, g_arena_points))

        except LocalizationUnavailableException:
            rospy.logwarn("Localization unavailable for camera frame '{0}' at time {1}".format(msg_frame, msg_time))


    all_3d_points = np.concatenate(points_3d)
    number_of_points = all_3d_points.shape[0]

    matched_areana_points = pt_match.match_points(all_3d_points, g_arena_points)

    try:
        (trans_fcu2arena, rot_fcu2arena) = get_tf_transform(
            global_state.configuration.frames["fcu"],
            global_state.configuration.frames["arena_center"],
            time,
            rospy.Duration(0, 500000000)
        )
        global_state.last_fcu_position = (trans_fcu2arena, rot_fcu2arena)
    except LocalizationUnavailableException:
        rospy.logerr("No FCU estimate at time {0}".format(time))
        no_estimate(time, global_state)
        return

    try:
        drone_pose = estimate_drone_pnp(
            intersections_2d,
            list_of_matches,
            camera_infos,
            global_state.configuration.frames["fcu"]
        )
    except LocalizationUnavailableException:
        try:
            drone_pose = estimate_drone_position_alone(
                all_3d_points,
                matched_areana_points,
                #time,
                global_state.last_fcu_position
            )
        except LocalizationUnavailableException:
            try:
                drone_pose = estimate_drone_simple(
                    all_3d_points,
                    matched_areana_points,
                    global_state.last_fcu_position
                )
            except LocalizationUnavailableException:
                rospy.logwarn("Not a single camera was able to detect an intersection!")
                no_estimate(time, global_state)
                return

    publish_fcu_transform(
        global_state,
        drone_pose[0],
        drone_pose[1],
        time
    )


def estimate_drone_pnp(point_list_2d, point_list_3d, camera_infos, fcu_frame):
    #type: (list[np.ndarray], list[np.ndarray], list[CameraInfo], str)->tuple[np.ndarray, quaternion.quaternion]
    bearings_list = []
    camera_rotation_list = np.empty((len(camera_infos), 3, 3),np.float)
    camera_translation_list = np.empty((len(camera_infos), 3),np.float)

    for i, (points_2d, camera_info) in enumerate(zip(point_list_2d, camera_infos)):
        camera_frame = camera_info.header.frame_id
        try:
            (trans_fcu2cam, rot_fcu2cam) = get_tf_transform(camera_frame, fcu_frame, camera_info.header.stamp, rospy.Duration.from_sec(0.01))
            camera_rotation_list[i,:,:] = quaternion.as_rotation_matrix(rot_fcu2cam)
            camera_translation_list[i,:] = trans_fcu2cam
        except LocalizationUnavailableException:
            continue

        image2camera_center = np.array([-camera_info.width/2.0, -camera_info.height/2.0, camera_info.K[0]])

        bearings = np.pad(points_2d, [(0, 0), (0, 1)], mode='constant') + image2camera_center
        bearings = pt_manip.normalize_vectors(bearings)

        bearings_list.append(bearings)#(quaternion.rotate_vectors(rot_fcu2cam, bearings))

    if len(bearings_list) == 0:
        raise LocalizationUnavailableException

    #point_list_3d[0][0, 2] = 0.1;
    fcu_pose_mat = opengv.epnp_multi_camera(bearings_list, point_list_3d, camera_translation_list, camera_rotation_list)
    if fcu_pose_mat is None:
        raise LocalizationUnavailableException
    fcu_pose_mat = fcu_pose_mat[0]

    if fcu_pose_mat[2,3] != fcu_pose_mat[2,3]:
        raise LocalizationUnavailableException

    rot = quaternion.from_rotation_matrix(fcu_pose_mat[0:3, 0:3])
    trans = fcu_pose_mat[:, 3]

    return trans, rot


def estimate_drone_position_alone(detected_3d_points, matched_3d_points, fcu_pose):
    # type: (np.ndarray, np.ndarray,tuple[np.ndarray, quaternion.quaternion])->tuple[np.ndarray, quaternion.quaternion]

    if detected_3d_points.size == 0:
        raise LocalizationUnavailableException

    #Par rapport au FCU
    detected_3d_points = pt_manip.transform_points_simple(detected_3d_points, -fcu_pose[0], fcu_pose[1].inverse())
    matched_3d_points = pt_manip.transform_points_simple(matched_3d_points, -fcu_pose[0], fcu_pose[1].inverse())

    tmp_publish("elikos_fcu", np.concatenate([detected_3d_points, matched_3d_points]))

    deltas_fcu_to_real = detected_3d_points - matched_3d_points
    dist = np.sqrt(np.sum(np.square(detected_3d_points), axis=1))

    m = -0.25

    weights_non_norm = np.maximum(dist * m + 1, 0)
    weights = np.true_divide(weights_non_norm, np.sum(weights_non_norm, axis=0))

    weighted_deltas = deltas_fcu_to_real * np.repeat(weights, 3, axis=0).reshape(deltas_fcu_to_real.shape)

    delta_p = np.sum(weighted_deltas, axis=0)

    if delta_p[0] != delta_p[0] or\
                    delta_p[1] != delta_p[1] or\
                    delta_p[2] != delta_p[2]:
        raise LocalizationUnavailableException

    return (fcu_pose[0] - delta_p, fcu_pose[1])


def estimate_drone_rigid_transform(detected_3d_points, matched_3d_points, time, fcu_pose):
    # type: (np.ndarray, np.ndarray, rospy.Time, tuple[np.ndarray, quaternion.quaternion])->tuple[np.ndarray, quaternion.quaternion]

    trans_fcu2arena = fcu_pose[0]
    rot_fcu2arena = fcu_pose[1]

    mask = np.ones(3, dtype=np.bool)
    mask[2] = False

    transform_arena_pts = matched_3d_points[:, mask] - np.array([trans_fcu2arena[0], trans_fcu2arena[1]])
    transform_detected_pts = detected_3d_points[:, mask] - np.array([trans_fcu2arena[0], trans_fcu2arena[1]])

    transform = cv2.estimateRigidTransform(
        pt_manip.prepare_points_for_cv(transform_detected_pts),
        pt_manip.prepare_points_for_cv(transform_arena_pts),
        False
    )

    if transform is None:
        raise LocalizationUnavailableException

    tmp_publish(
        global_state.configuration.frames["arena_center"],
        np.concatenate([transform_arena_pts, transform_detected_pts])
    )

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

    mean = np.mean(detected_3d_points, axis=0)

    delta_rot = quaternion.from_euler_angles(0, 0, -angle_delta)

    trans = trans_fcu2arena + np.array((dx, dy, -mean[2]))

    return (trans, delta_rot * rot_fcu2arena)


def estimate_drone_simple(points_in_3d, matched_points_in_3d, fcu_pose):
    if points_in_3d.shape[0] > 0:
        dt = np.mean(matched_points_in_3d - points_in_3d, axis=0)
        return (fcu_pose[0] + dt, fcu_pose[1])
    else:
        raise LocalizationUnavailableException

def transform_points(input_points_3d, input_points_frame, dest_frame, frame_time):
    (trans_ref2dst, rot_ref2dst) = get_tf_transform(
        input_points_frame,
        dest_frame,
        frame_time,
        rospy.Duration(0, 5000000)#5ms
    )

    input_points_3d = quaternion.rotate_vectors(rot_ref2dst, input_points_3d)
    input_points_3d += trans_ref2dst

    return input_points_3d


def publish_fcu_transform(global_state, trans, rot, frame_time):
    # type: (GlobalState, np.ndarray, quaternion.quaternion, rospy.Time)->None

    global g_tf_broadcaster

    g_tf_broadcaster.sendTransform(
        trans,
        pt_manip.create_tf_from_quaterion(rot),
        frame_time,
        global_state.configuration.frames["output"],
        global_state.configuration.frames["arena_center"]
    )


def tmp_publish(frame, camera_points):
    global g_arena_points
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
    output_message.header.frame_id = frame

    g_pub_dbg.publish(output_message)


def get_tf_transform(source_frame, dest_frame, time, timeout):
    # type: (str, str, rospy.Time, rospy.Duration)->(np.ndarray, quaternion.quaternion)
    global g_tf_listener
    try:
        g_tf_listener.waitForTransform(dest_frame, source_frame, time, timeout)
    except tf.Exception as e:
        raise LocalizationUnavailableException(message="Tf lookup failed", cause=e)

    (trans, rot) = g_tf_listener.lookupTransform(dest_frame, source_frame, time)
    return np.array(trans), pt_manip.create_quaterion_from_tf(rot)


def publish_fcu_if_no_pos():
    pass


def init_node():
    # type: ()->GlobalState
    """
    Initialises the node.
    """
    global g_tf_listener, g_pub_dbg, g_tf_broadcaster
    rospy.init_node("feature_tracking")

    global_state = GlobalState()

    rospy.loginfo("Publishing on %s", global_state.configuration.frames["output"])

    g_tf_listener = tf.TransformListener()
    g_tf_broadcaster = tf.TransformBroadcaster()

    g_pub_dbg = rospy.Publisher("/localization/features_debug", PoseArray, queue_size=10)

    #Read params from the parameter server
    return global_state


def start_listening_for_localization(global_state):
    if start_listening_for_localization.inited is False:
        print "Listen started"
        start_listening_for_localization.inited = True
        global_state.synchonyser.registerCallback(
            input_localization_points,
            global_state
        )
start_listening_for_localization.inited = False


###
#
# State machine
#
###
def state_init(global_state, time_since_state_begin):
    # type: (GlobalState, rospy.Duration)->function

    publish_fcu_transform(
        global_state,
        global_state.configuration.initial_drone_position,
        global_state.configuration.initial_drone_rotation,
        rospy.Time.now()
    )
    try:
        global_state.last_fcu_position = get_tf_transform(
            global_state.configuration.frames["fcu"],
            global_state.configuration.frames["arena_center"],
            rospy.Time.now(),
            rospy.Duration(1)
        )
    except LocalizationUnavailableException:
        rospy.loginfo("no fcu at time {0}".format(rospy.Time.now()));
        global_state.last_fcu_position = None

    if global_state.last_fcu_position is not None:
        return state_stablization
    else:
        return state_init


def state_stablization(global_state, time_since_state_begin):
    # type: (GlobalState, rospy.Duration)->function

    publish_fcu_transform(
        global_state,
        global_state.configuration.initial_drone_position,
        global_state.configuration.initial_drone_rotation,
        rospy.Time.now()
    )
    if time_since_state_begin.to_sec() > global_state.configuration.stabilization_time:
        return state_climb
    else:
        return state_stablization


def state_climb(global_state, time_since_state_begin):
    # type: (GlobalState, rospy.Duration)->function
    #TODO this state
    start_listening_for_localization(global_state)
    return state_wait_for_message


def state_wait_for_message(global_state, time_since_state_begin):
    if global_state.total_messages_processed > 0:
        return state_normal
    elif time_since_state_begin.to_sec() > 2:
        return state_warn_no_messages
    else:
        return state_wait_for_message


def state_warn_no_messages(global_state, time_since_state_begin):
    rospy.logwarn("No messagess were yet processed. Check camera pipeline. Rechecking in 2 seconds")
    return state_wait_for_message


def state_normal(global_state, time_since_state_begin):
    # type: (GlobalState, rospy.Duration)->function
    #TODO this state

    # watchdog
    duration_since_last_message = (rospy.Time.now() - global_state.last_message_time).to_sec()
    if duration_since_last_message > global_state.configuration.watchdog_max_message_delay:
        rospy.logerr("No messages processed in the last {0} seconds".format(duration_since_last_message))


    return state_normal



def run_state_machine(global_state):
    # type: (GlobalState)->None
    current_state = state_init

    last_state_change_time = rospy.Time.now()

    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        next_state = current_state(
            global_state,
            rospy.Time.now() - last_state_change_time
        )
        if next_state is not current_state:
            last_state_change_time = rospy.Time.now()
            current_state = next_state
            rospy.logdebug("State is now %s", next_state)
        try:
            r.sleep()
        except rospy.ROSInterruptException:
            pass # Ros should be shutdown


###node_configuration
#
# Globals
#
###
g_tf_listener = None
g_tf_broadcaster = None

g_arena_points = pt_match.create_grid_mesh(side_points_number=21, side_mesure=20) - np.array([10, 10, 0])



if __name__ == '__main__':
    global_state = init_node()

    g_arena_points = pt_match.create_grid_mesh(
        side_mesure=rospy.get_param("~arena_size", 20),
        side_points_number=rospy.get_param("~arena_intersection_num", 21)
    )

    initial_drone_position = np.array(
        rospy.get_param("~initial_drone_pos", [0, 0, 0])
    )


    initial_drone_rotation = quaternion.as_quat_array(
        np.array(
            rospy.get_param("~initial_drone_rot", [1, 0, 0, 0])
        )
    )

    run_state_machine(global_state)

    rospy.spin()




