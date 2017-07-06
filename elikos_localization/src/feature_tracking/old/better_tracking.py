#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Ce module python s'occupe de suivre le drone et d'envoyer des estimés de position et de rotation
angulaire au pixhawk.
"""

import math

import numpy as np
import unscented_kalman_filter as ukf

import rospy

import cv2

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import MerweScaledSigmaPoints

import quaternion

from scipy.linalg import block_diag

#==============================================================================
# Kalman filter equations
#==============================================================================

def f(x, dt):
    """
    State transition function of the filter.
    """
    mini_f = np.array([[1, dt],
                       [0, 1]])
    F = block_diag(mini_f, mini_f, mini_f, mini_f)
    return np.matmul(F, x)

def h_pose(x):
    """
    Mesurement convertion function.
    Note: z is [theta, x, y, z]
    points to match.
    """
    return np.take(x, [0, 2, 4, 6])

def normalize_angle(x):
    """
    Moves an angle into the renge [-pi, pi)
    """
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x

def compute_residuals_x(a, b):
    """
    Computes the residual in the state space
    """
    y = a - b
    y[0] = normalize_angle(y[0])
    return y

def compute_mean_x(sigmas, Wm):
    """
    Computes the mean in the state space
    """
    x = np.empty(8)

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 0]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 0]), Wm))
    x[0] = math.atan2(sum_sin, sum_cos)
    x[1:8] = np.sum(np.dot(sigmas[1:8, 1:8], Wm))
    return x

def create_Q(dt, intensity):
    mini_q = Q_discrete_white_noise(dim=2, dt=dt, var=intensity)
    mini_q = mini_q #+ np.array([[0, 0],[0, 10]])
    return block_diag(mini_q, mini_q, mini_q, mini_q)

#==============================================================================
# Custom filtering functions
#==============================================================================
def filter_pose(filter, z, R, dt, model):
    filter.predict(dt=dt)
    predicted_state = filter.x
    filter.P *= 1.01

    drone_planar_position = state_position(predicted_state)[0:2]

    positions_relative_to_drone = model.features_positions - np.tile(drone_planar_position, (model.number_of_features,1))
    positions_in_drone_space = np.matmul(positions_relative_to_drone, state_rotation_matrix(predicted_state).T)

    matched_estimation = np.empty((1, 0, 2))
    matched_mesurement = np.empty((1, 0, 2))
    for i in xrange(0, len(z), 3):
        position = z[i:i+2]
        closest_point_index = closest_point(positions_in_drone_space, position)
        matched_estimation = np.append(matched_estimation, np.array([[positions_in_drone_space[closest_point_index]]]), axis=1)
        matched_mesurement = np.append(matched_mesurement, np.array([[position]]), axis=1)

    transform = cv2.estimateRigidTransform(matched_estimation.astype(np.float32), matched_mesurement.astype(np.float32), False)

    if transform is None:
        rospy.logerr("The transformation was null! Skipping set.")
        return#TODO this will die, needs to update

    angle = math.atan2(transform[0,0], transform[1,0])
    predicted_angle = state_yaw(predicted_state)

    angle_delta = normalize_angle(angle - predicted_angle)
    if angle_delta > 3 * math.pi / 4:
        angle -= math.pi
    elif angle_delta > math.pi / 4:
        angle -= math.pi / 2
    elif angle_delta < - 3 * math.pi / 4:
        angle +=math.pi
    elif angle_delta < - math.pi / 4:
        angle += math.pi / 2

    scale = math.sqrt(transform[0,0]**2 + transform[1,0] ** 2)
    x = transform[0, 2]
    y = transform[1, 2]
    altitude = z[2]
    
    if scale < 0.8 or scale > 1.2:
        rospy.logwarn("Scale should be 1 and is {0}".format(scale))

    mesure = np.array([-angle, state_position(predicted_state)[0] - x, state_position(predicted_state)[1] - y, -altitude])
    model.lastMesuredStatus = mesure

    filter.update(z=mesure, R=R, hx_args=())

#==============================================================================
# Ros interraction functions
#==============================================================================

def input_pose_array(pose_array, extra_args):
    model, publisher = extra_args

    current_time = pose_array.header.stamp.to_sec()

    z_size = 3 * len(pose_array.poses)
    z = np.empty((z_size,))
    mini_r = np.array([[0.1, 0, 0],
                       [0, 0.1, 0],
                       [0, 0, 0.1]])
    mini_r_array = []

    for i, pose in enumerate(pose_array.poses):
        z[i * 3 + 0] = pose.position.x
        z[i * 3 + 1] = pose.position.y
        z[i * 3 + 2] = pose.position.z
        mini_r_array.append(mini_r)

    R = np.array([[0.1, 0, 0, 0],
                  [0, 0.1, 0, 0],
                  [0, 0, 0.1, 0],
                  [0, 0, 0, 0.1]])

    model.tracker.calculate_for_new_message(
        z,
        R,
        current_time,
        "pose_array",
        filtering_function=filter_pose,
        filtering_function_args=(model,))
    publish_current_status(model, publisher, pose_array.header.stamp)

def publish_current_status(model, publisher, time_stamp):
    message = PoseStamped()
    current_state = model.tracker.x
    
    #position = None
    #orientation = None
    #if model.lastMesuredStatus is None:
    position = state_position(current_state)
    orientation = state_orientation_quaternion(current_state)
    #else:
    #    position = model.lastMesuredStatus[1:]
    #    orientation = quaternion.from_euler_angles(model.lastMesuredStatus[0], 0.0, 0.0)

    message.pose.position.x = -position[1]#TODO remplacer par des transforms.
    message.pose.position.y = -position[0]#TODO remplacer par des transforms.
    message.pose.position.z = position[2]

    message.pose.orientation.w = orientation.w
    message.pose.orientation.x = orientation.y
    message.pose.orientation.y = orientation.x
    message.pose.orientation.z = orientation.z

    message.header.stamp = time_stamp
    message.header.frame_id = "elikos_arena_origin"

    publisher.publish(message)

#==============================================================================
# State manipulation functions
#==============================================================================
def state_position(x):
    return x[2::2]

def state_yaw(x):
    return x[0]

def state_rotation_matrix(x):
    cs = math.cos(state_yaw(x))
    ss = math.sin(state_yaw(x))

    R = np.array([[cs, -ss],
                  [ss, cs]])
    return R

def state_orientation_quaternion(x):
    return quaternion.from_euler_angles(state_yaw(x), 0.0, 0.0)

#==============================================================================
# Other methods
#==============================================================================

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

def get_param(name, default):
    """
    Shortcut for a ros parameter fetch.
    """
    return rospy.get_param(name) if rospy.has_param(name) else default


def start():
    """
    Main function. Runs the node.
    """
    arena_model = Tracker(21, 20, np.array([10, 10]))

    point_R_matrix = np.array(get_param("point_R_matrix", [1, 0, 0, 0, 1, 0, 0, 0, 1]))
    imu_R_matrix = np.array(get_param("imu_R_matrix", [1, 0, 0, 0, 1, 0, 0, 0, 1]))

    point_R_matrix = np.reshape(point_R_matrix, (3,3))
    imu_R_matrix = np.reshape(imu_R_matrix, (3,3))

    imu_input_topic = get_param("imu_input_topic", "/mavros/imu/data_raw")
    pose_input_topic = get_param("pose_input_topic", "/localization/features")
    drone_pose_output_topic = get_param("drone_pose_output_topic", "localization/drone/pose")

    rospy.init_node("feature_tracking", anonymous=False)

    pub = rospy.Publisher(drone_pose_output_topic, PoseStamped, queue_size=4)

    rospy.Subscriber(
        pose_input_topic,
        PoseArray,
        callback=input_pose_array,
        callback_args=(arena_model, pub),
        queue_size=20)

    #rospy.Subscriber(
    #    imu_input_topic,
    #    Imu, callback=input_imu_data,
    #    callback_args=(arena_model, pub),
    #    queue_size=20)

    rospy.spin()

#==============================================================================
# Classes
#==============================================================================
class Tracker:
    def __init__(self, number_of_points, size, arena_origin_position):
        self.tracker = ukf.MultiUnscentedKalmanFilter(
            np.array([0, 0, 0, 0, 0, 0, 2, 0]),
            np.eye(8) * 500,
            create_Q,
            Q_generator_args=(0.1,)
        )
        self.tracker.filters["pose_array"] = ukf.UnscentedKalmanFilter(
            8,
            4,
            0,
            h_pose,
            f,
            MerweScaledSigmaPoints(8, 1, 3, -5)
        )


        stride = size / (number_of_points - 1.0)
        self.number_of_features = number_of_points * number_of_points
        self.features_positions = np.empty((self.number_of_features, 2))
        for x in xrange(number_of_points):
            for y in xrange(number_of_points):
                self.features_positions[x + y * number_of_points] = np.array(
                    [x*stride, y*stride]
                ) - arena_origin_position
        
        self.lastMesuredStatus = None

if __name__ == '__main__':
    start()


