#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Ce module python s'occupe de suivre les points d'intetêts de l'arène avec un filtre de Kalman
"""
from datetime import datetime
import copy
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import cv2

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
import numpy as np



def create_F(dt):
    """
    Creates F for the kalman filter.
    @param dt the delta time
    @return the F matrix
    @rtype a numpy array
    """
    return np.array([[1,dt, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [0, 0, 1,dt, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1,dt],
                     [0, 0, 0, 0, 0, 1]])


def create_Q(dt):
    """
    Creates a simple Q matrix for the kalman filter.
    @param dt the delta time
    @return the Q matrix
    @rtype a numpy array
    """
    q = Q_discrete_white_noise(dim=2, dt=dt, var=0.001)
    return block_diag(q, q, q)

def create_B(dt):
    """
    Creates the B matrix, that takes the acceleration and gives the change to the velocity and
    position
    @param dt the variation in time in seconds
    """
    tt = dt * dt / 2
    return np.array([
        [0, 0, 0],
        [dt, 0, 0],
        [0, 0, 0],
        [0, dt, 0],
        [0, 0, 0],
        [0, 0, dt]
    ])

def create_tracker(initial_state):
    """
    Creates a simple Kalman filter
    @param initial_state the initial state of the system. Of the form [r_x, v_x, r_y, v_y, r_z, v_z]
    """
    tracker = KalmanFilter(6, 3, dim_u=3)

    tracker.H = np.array([[1, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0]])
    tracker.F = create_F(0)
    tracker.Q = create_Q(0)
    tracker.H = np.array([[1, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0]])
    tracker.R = np.array([[5., 0, 0],
                          [0, 5 ,0],
                          [0, 0 ,5]])
    tracker.B = create_B(0)

    tracker.x = initial_state.T
    tracker.P = np.eye(6) * 500.
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


last_time = 0.0
first_call = True
global_acceleration = np.zeros((3,))

def input_imu_data(imu_data):
    global global_acceleration
    global_acceleration[0] = imu_data.linear_acceleration.x
    global_acceleration[1] = imu_data.linear_acceleration.y
    global_acceleration[2] = imu_data.linear_acceleration.z

def input_pose_array(pose_array, extra_args):
    """
    Ros callback for the input pose arrays.
    @param pose_array The array that contains all position for the features.
    @param extra_args A tuple that contains the model and the publisher
    """
    
    global last_time, first_call, global_acceleration
    
    model, publisher = extra_args

    current_time = pose_array.header.stamp.to_sec()
    delta_time = current_time - last_time
    last_time = current_time

    if first_call:
        delta_time = 0
        first_call = False

    print delta_time
    model.update_trackers_prediction(delta_time, global_acceleration)
    print delta_time

    predicted = model.get_all_trackers()[:]
    predicted_pos = model.get_all_trackers_position()[:]
    predicted_state_positions = np.take(model.get_tracker_x(), [0, 2, 4], axis=1)

    associated_trackers = []
    average = np.array([0.,0,0])
    average_in = np.zeros((3,))
    average_out = np.zeros((3,))


    time_start = datetime.now()
    for pose in pose_array.poses:
        average += np.array([pose.position.x, pose.position.y, pose.position.z])

        currentPosition = np.array([pose.position.x, pose.position.y, pose.position.z])
        tracker_index = closestNode(predicted_state_positions, currentPosition)

        if tracker_index is None:
            print "Too many points"
            break

        in_position = predicted_pos[tracker_index]
        average_in += np.array(in_position)
        average_out += np.array([pose.position.x, pose.position.y, pose.position.z])

        associated_trackers.append((predicted[tracker_index], in_position, pose))
        del predicted[tracker_index]
        del predicted_pos[tracker_index]

        predicted_state_positions = np.delete(predicted_state_positions, tracker_index, axis=0)

    time_end = datetime.now()

    time_delta = time_end - time_start
    print "Time elapsed = {0}".format(time_delta)


    average /= len(pose_array.poses)
    average_in /= len(associated_trackers)
    average_out /= len(associated_trackers)

    deplacement = average_out - average_in

    point_in_matrix = np.empty((len(associated_trackers),3))
    point_out_matrix = np.empty((len(associated_trackers),3))


    for i, tpp in enumerate(associated_trackers):
        tracker, position, pose = tpp
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])

        tracker.update(pos.T)

        point_in_matrix[i] = np.array(position)
        point_out_matrix[i] = np.array(pos)


    print deplacement

    #update non-asociated trackers
    for i, tracker in enumerate(predicted):
        position = np.array(predicted_pos[i])
        #position = np.append(position, 1)#passing to homogenious coordinates
        final_position = deplacement + position
        tracker.update(final_position)


    #Créer le message et le publier
    output_message = PoseArray()

    for tracker in model.get_all_trackers():
        p = Pose()
        p.position.x = tracker.x[0]
        p.position.y = tracker.x[2]
        p.position.z = tracker.x[4]

        output_message.poses.append(p)

    output_message.header = Header()
    output_message.header.stamp = pose_array.header.stamp

    publisher.publish(output_message)
    #print "Average displacement : {0}".format(speedMean)

class ArenaModel:
    def __init__(self, number_of_points, size, max_accepted_error=1):
        self.max_accepted_error = max_accepted_error
        stride = size / (number_of_points - 1.0)

        self.trackers = []
        self.tracker_list = []
        self.tracker_pos_list = []

        for x in xrange(number_of_points):

            trackers = []
            self.trackers.append(trackers)

            for y in xrange(number_of_points):
                tracker = create_tracker(
                    np.array(
                        [x*stride, 0, y*stride, 0, 0 ,0]
                    )
                )
                trackers.append(tracker)
                self.tracker_list.append(tracker)
                self.tracker_pos_list.append((x,y,0))


    def get_all_trackers(self):
        """
        @return a linear list of all trackers in the current model.
        @rtype a mutable list of KalmanFilters
        """
        return self.tracker_list

    def get_all_trackers_position(self):
        """
        @return a list of position of trackers on the grid following the same order
        as in the tracker list form get all trackers
        @rtype a list of coordinates (tuples)
        """
        return self.tracker_pos_list

    def get_tracker(self, x, y):
        """
        @return the tracker (as a mutable) that is at the position (x,y) on the grid
        @rtype filterpy.kalman.KalmanFilter
        """
        return self.trackers[x][y]


    def update_trackers_prediction(self, delta_time, u):
        """
        Updates the F and Q matrix of all trackers using the provided delta time.
        @param delta_time the elaplsed time to update the trackers with
        @param u the controll parameters
        """
        newF = create_F(delta_time)
        newQ = create_Q(delta_time)
        newB = create_B(delta_time)
        print np.matmul(newB, u)
        for tracker in self.get_all_trackers():
            tracker.F = newF
            tracker.Q = newQ
            tracker.B = newB
            tracker.predict(u)
    
    def get_tracker_x(self):
        """
        @return the values of the state of each tracker
        @rtype a ndarray fo dim 2 [x_1, x_2, ...]
        """
        #TODO 6 is a magic number
        tracker_x = np.empty((len(self.tracker_list), 6))
        for i, tracker in enumerate(self.tracker_list):
            tracker_x[i] = tracker.x
        return tracker_x



def talker():
    """ Main func. """
    arena_model = ArenaModel(21, 20)
    rospy.init_node("talker")

    pub = rospy.Publisher("arena_static_feature_pose", PoseArray, queue_size=4)

    rospy.Subscriber("arena_features", PoseArray, callback=input_pose_array, callback_args=(arena_model, pub), queue_size=20)
    rospy.Subscriber("imu/data_raw", Imu, callback=input_imu_data, queue_size=20)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

