#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Ce module python s'occupe de suivre les points d'intetêts de l'arène avec un filtre de Kalman
"""
from datetime import datetime
from math import sqrt
import copy
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import cv2

from filterpy.kalman import KalmanFilter
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from filterpy.common import dot3

from scipy.linalg import block_diag

import numpy as np
from numpy.linalg import inv
import quaternion

import unscented_kalman_filter as ukf

#####
#### For testing
#####
test_values = dict()
def dump_test():
    global test_values
    for key, val in test_values.iteritems():
        np.save("./{0}".format(key), val)
def log_value(**kwargs):
    global test_values
    for key, val in kwargs.iteritems():
        if key in test_values:
            test_values[key] = np.append(test_values[key], val)
        else:
            test_values[key] = np.array([val])


#####
#### Global variables
#####
gravity = np.array([0, 0, -9.81])

def f_state(x, dt):
    angular_speed = x[4:7] * dt
    rotation_this_frame = quaternion.from_rotation_vector(angular_speed)
    rotation = quaternion.as_quat_array(x[0:4])
    rotation = rotation * rotation_this_frame

    tt = (dt * dt)/2
    mini_f = np.array([[1, dt, tt],
                       [0, 1, dt],
                       [0, 0, 1]])
    fMat = block_diag(np.zeros((4,4)), np.identity(3), mini_f, mini_f, mini_f)
    x_out = np.matmul(fMat, x)
    x_out[0:4] = quaternion.as_float_array(rotation)
    return x_out

def h_imu(x):
    u"""
    Transforms the state X into the mesurement space of the imu
    Parameters:
    -----------
        x : the state of the system
    IMU mesurement space:
    ---------------------
        z = [w, a]
        where w is the angular speed
        and a is the linear acceleration
    """
    global gravity

    rot = quaternion.as_quat_array(x[0:4])
    irot = np.conjugate(rot)
    accel = quaternion.rotate_vectors(irot, (x[13:] + gravity))

    angular_speed = x[4:7]
    return np.concatenate((angular_speed, accel))

def h_pose(x, p):
    rot = quaternion.as_quat_array(x[0:4])
    delta_r = x[7:10]
    new_pos = p + delta_r
    z = quaternion.rotate_vectors(rot, new_pos, axis=2)
    return z

def h_magnetometer(x):
    pass

def h_lidar(x):
    pass

def h_optical_flow(x):
    pass

def h_point_cloud(x):
    pass

def create_F(dt):
    """
    Creates F for the kalman filter.
    @param dt the delta time
    @return the F matrix
    @rtype a numpy array
    """
    tt = (dt * dt)/2
    mini_f = np.array([[1,dt,tt],
                       [0, 1,dt],
                       [0, 0, 1]])
    return block_diag(mini_f, mini_f, mini_f)

def create_Q(dt, variation):
    """
    Creates a simple Q matrix for the kalman filter.
    @param dt the delta time
    @return the Q matrix
    @rtype a numpy array
    """
    rot_q = np.eye(7) * variation
    q = Q_discrete_white_noise(dim=3, dt=dt, var=variation)
    return block_diag(rot_q, q, q, q)

def create_tracker(initial_state):
    """
    Creates a simple Kalman filter
    @param initial_state the initial state of the system. Of the form [r_x, v_x, a_x, r_y, v_y, a_y, r_z, v_z, a_z]
    """
    tracker = KalmanFilter(9, 3, dim_u=3)

    tracker.F = create_F(0)
    tracker.Q = create_Q(0, 1)

    tracker.x = initial_state.T
    tracker.P = np.eye(9) * 500.
    return tracker

def closestNode(node_array, position):
    """
    @param node_array A numpy array of positions size:(x, 3)
    @param position A numpy vector size:(3,)
    @return the index of the closest node in the array of input nodes
    @rtype ndarray(int)
    """
    if node_array.shape[0] is 0:
        return None
    deltas = node_array - position
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

def publish_current_status(model, publisher, time_stamp):
    """
    Published the current status of the model on the publisher, with the given time stamp
    Parameters:
    -----------
        model: the model
        publisher: the publisher
        time_stamp: the time stamp (ros header.stamp)
    """
    output_message = PoseArray()

    point = model.get_drone_position()

    p = Pose()
    p.position.x = -point[0]
    p.position.y = -point[1]
    p.position.z = -point[2]

    output_message.poses.append(p)

    output_message.header = Header()
    output_message.header.stamp = time_stamp
    output_message.header.frame_id = "elikos_local_origin"

    publisher.publish(output_message)



def get_delta_time(current_time):
    """
    Time keeper. Call with current time to get the delta-time of the last time this function was called.
    Parameters:
    -----------
        current_time: the current time in seconds
    """

    delta_time = current_time - get_delta_time.last_time
    get_delta_time.last_time = current_time

    if get_delta_time.first_call:
        delta_time = 0
        get_delta_time.first_call = False
    
    return delta_time
get_delta_time.last_time = 0.0
get_delta_time.first_call = True

def input_imu_data(imu_data, extra_args):
    """
    Ros callback for the input imu data
    Parameters:
    -----------
        imu_data : the data object sent by the Imu
        extra_args : A tuple that contains the model and the publisher
    """
    model, publisher = extra_args
    delta_time = get_delta_time(imu_data.header.stamp.to_sec())
    #print "Acceleration dt : {0}".format(delta_time)
    model.tracker.set_active("imu")

    model.predict(delta_time)


    measurement = np.empty((6,))
    measurement[0] = imu_data.angular_velocity.x
    measurement[1] = imu_data.angular_velocity.y
    measurement[2] = imu_data.angular_velocity.z
    measurement[3] = imu_data.linear_acceleration.x
    measurement[4] = imu_data.linear_acceleration.y
    measurement[5] = imu_data.linear_acceleration.z
    
    R_angular = np.array(imu_data.angular_velocity_covariance)
    R_linear = np.array(imu_data.linear_acceleration_covariance)

    R_angular = np.reshape(R_angular, (3,3))
    R_linear = np.reshape(R_linear, (3,3))

    #print "Acceleration val: {0}".format(accel)
    R = block_diag(R_angular, R_linear)

    model.tracker.update(measurement, R=R)
    #model.variate_q()
    log_value(acceleration=np.take(model.tracker.x, np.array([2, 5, 8])))
    log_value(accel_x=model.tracker.x[2])
    log_value(accel_y=model.tracker.x[5])
    log_value(accel_z=model.tracker.x[8])
    #log_value(acceleration_residuals=dot3(model.tracker.y.T, inv(model.tracker.S), model.tracker.y))
    publish_current_status(model, publisher, imu_data.header.stamp)


def input_pose_array(pose_array, extra_args):
    """
    Ros callback for the input pose arrays.
    @param pose_array The array that contains all position for the features.
    @param extra_args A tuple that contains the model and the publisher
    """
    global test_time, test_residuals
    model, publisher = extra_args

    current_time = pose_array.header.stamp.to_sec()
    delta_time = get_delta_time(current_time)

    
    model.predict(delta_time)

    predicted_positions = model.get_feature_position_relative_to_drone()
    z = np.array([])
    
    number_of_asscociated_points = 0

    drone_position = model.get_drone_position()
    for pose in [pose_array.poses[0]]:

        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        position_index = closestNode(predicted_positions, current_position)

        if position_index is None:
            print "Too many points"
            break

        old_position = predicted_positions[position_index]
        new_position = current_position
        displacement = (old_position - new_position)
        new_drone_position = drone_position + displacement

        z = np.append(z, new_drone_position)
        number_of_asscociated_points += 1

    mini_R = model.point_R_matrix
    mini_R_list = [mini_R] * number_of_asscociated_points
    R = block_diag(*mini_R_list)
    mini_H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 1, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 1, 0, 0]])

    H = np.tile(mini_H, (number_of_asscociated_points, 1))

    model.tracker.update(z, R=R, H=H)
    model.variate_q()

    log_value(test_residuals=model.tracker.y[1])

    publish_current_status(model, publisher, pose_array.header.stamp)

class ArenaModel:
    def __init__(self, number_of_points, size, max_accepted_error=1):
        self.max_accepted_error = max_accepted_error
        stride = size / (number_of_points - 1.0)
        self.number_of_features = number_of_points * number_of_points

        self.q_variation_count = 0
        self.q_scale_factor = 2.5
        self.epsilon_max = 0.1

        #create_tracker(np.array([0,0,0, 0,0,0, 0,0,0]))
        self.tracker = ukf.MultiUnscentedKalmanFilter(
            np.array([1,0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0]), np.eye(16) * 500
        )
        self.tracker.filters["imu"] = ukf.UnscentedKalmanFilter(
            16,
            6,
            0,
            h_imu,
            f_state,
            MerweScaledSigmaPoints(16, 0.5, 2, -13)
        )
        self.tracker.filters["pose_array"] = ukf.UnscentedKalmanFilter(
            16,
            3,
            0,
            h_pose,
            f_state,
            MerweScaledSigmaPoints(16, 0.5, 2, -13)
        )
        self.tracker.set_active("imu")

        self.features_positions = np.empty((self.number_of_features, 3))

        self.Q_matrix_variation = 0.001
        self.point_R_matrix = np.array([0,0,0,0,0,0,0,0,0])
        self.imu_R_matrix = np.array([0,0,0,0,0,0,0,0,0])

        for x in xrange(number_of_points):
            for y in xrange(number_of_points):
                self.features_positions[x + y * number_of_points] = np.array(
                    [x*stride, y*stride, 0]
                )
        

    def variate_q(self):#don't use
        y = self.tracker.active.y
        eps = dot3(y.T, inv(self.tracker.active.S), y)
        if eps > self.epsilon_max:
            self.q_variation_count += 1
        elif self.q_variation_count > 0:
            self.q_variation_count -= 1

    def get_drone_position(self):
        """
        Returns:
        --------
        ndarray(float) : 3x1 array representing the position of the drone
        """
        return np.take(self.tracker.active.x, np.array([0, 3, 6]))
    
    def get_feature_position_relative_to_drone(self):
        """
        Returns a numpy array containing numpy arrays of position of features on the map.
        """
        position = self.get_drone_position()
        return self.features_positions - np.tile(position, (self.number_of_features,1))

    def predict(self, delta_time):
        """
        Calculates a prediction based on the data of the filter and the time
        Parameters:
        -----------
            delta_time: the time difference to use to predict the state
        """
        F = create_F(delta_time)
        Q = create_Q(delta_time, self.Q_matrix_variation * (self.q_scale_factor ** self.q_variation_count))
        self.tracker.active.Q=Q
        self.tracker.active.predict(dt=delta_time)


def get_param(name, default):
    return rospy.get_param(name) if rospy.has_param(name) else default

def talker():
    """ Main func. """
    arena_model = ArenaModel(21, 20)
    rospy.init_node("feature_tracking")

    point_R_matrix = np.array(get_param("~point_R_matrix", [1, 0, 0, 0, 1, 0, 0, 0, 1]))
    imu_R_matrix = np.array(get_param("~imu_R_matrix", [1, 0, 0, 0, 1, 0, 0, 0, 1]))
    arena_model.Q_matrix_variation = get_param("~Q_matrix_variation", 0.001)

    imu_input_topic = get_param("~imu_input_topic", "imu/data_raw")
    pose_input_topic = get_param("~pose_input_topic", "arena_features")
    drone_pose_output_topic = get_param("~drone_pose_output_topic", "arena_static_feature_pose")

    point_R_matrix = np.reshape(point_R_matrix, (3,3))
    imu_R_matrix = np.reshape(imu_R_matrix, (3,3))
    
    arena_model.point_R_matrix = point_R_matrix
    arena_model.imu_R_matrix = imu_R_matrix
    print arena_model.point_R_matrix
    print arena_model.imu_R_matrix


    pub = rospy.Publisher(drone_pose_output_topic, PoseArray, queue_size=4)

    rospy.Subscriber(
        pose_input_topic,
        PoseArray,
        callback=input_pose_array,
        callback_args=(arena_model, pub),
        queue_size=20)

    rospy.Subscriber(
        imu_input_topic,
        Imu, callback=input_imu_data,
        callback_args=(arena_model, pub),
        queue_size=20)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    dump_test()


