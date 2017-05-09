#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Ce module python s'occupe de suivre les points d'intetêts de l'arène avec un filtre de Kalman
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray

from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
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

def create_tracker(initial_state):
    """
    Creates a simple Kalman filter
    @param initial_state the initial state of the system. Of the form [r_x, v_x, r_y, v_y, r_z, v_z]
    """
    tracker = KalmanFilter(6, 3)

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

    tracker.x = initial_state.T
    tracker.P = np.eye(6) * 500.
    return tracker

def input_pose_array(pose_array, model):
    """ Ros callback for the input pose arrays. """
    
    for tracker in model.trackers:
        tracker.predict()
    predicted = list(model.trackers)

    associated_trackers = []

    for pose in pose_array.poses:
        min_distance = float("inf")
        tracker_index = -1
        for i in xrange(len(predicted)):
            state = predicted[i]
            distance = (pose.position.x - state.x[0]) ** 2 + (pose.position.y - state.x[3]) ** 2 + (pose.position.z - state.x[5]) ** 2
            if distance < min_distance:
                min_distance = distance
                tracker_index = i
        if tracker_index is -1:
            print "Too many features!"
            break
        
        associated_trackers.append(predicted[tracker_index])
        del predicted[tracker_index]

    for i in xrange(len(associated_trackers)):
        pos = pose_array.poses[i].position
        associated_trackers[i].update(np.array([pos.x, pos.y, pos.z]).T)
    
    speedMean = [0., 0, 0]
    for tracker in model.trackers:
        speedMean[0] += tracker.x[1]
        speedMean[1] += tracker.x[3]
        speedMean[2] += tracker.x[5]
    
    for i in xrange(len(speedMean)):
        speedMean[i] /= len(model.trackers)

    print "Average displacement : {0}".format(speedMean)

class ArenaModel:
    def __init__(self, size):
        self.trackers = [
            create_tracker(
                np.array(
                    [x*2, 0, y*2, 0, 0 ,0]
                )
            )
            for x in xrange(size)
            for y in xrange(size)
        ]
        self.trackers[0].S

def talker():
    """ Main func. """
    arena_model = ArenaModel(5)
    rospy.init_node("talker")

    pub = rospy.Publisher("arena_static_feature_pose", PoseArray, queue_size=4)

    rospy.Subscriber("arena_features", PoseArray, callback=input_pose_array, callback_args=arena_model, queue_size=20)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

