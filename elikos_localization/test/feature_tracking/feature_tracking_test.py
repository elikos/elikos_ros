#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Module de text pour le module feature_tracking d'elikos_localization
"""
import time
from datetime import datetime
import threading

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

import numpy as np
from numpy.random import random_sample


class ArenaIntersectionModel(object):
    """ Arena model for testing. """

    def __init__(self, width, height, arena_size=20):
        self.size = width * height
        self.linear_velocity = np.zeros((3,))
        self.linear_acceleration = np.zeros((3,))
        x_components = np.linspace(0, arena_size, width)
        y_components = np.linspace(0, arena_size, height)
        z_components = np.array([0])

        xv, yv, zv = np.meshgrid(x_components, y_components, z_components)
        self.points = np.reshape(np.dstack((xv, yv, zv)), (self.size, 3))


    def get_points(self, error):
        return self.points + ((random_sample((self.size, 3)) - 0.5) * 2 * error )

    def get_linear_acceleration(self, error):
        return self.linear_acceleration + ((random_sample((3,)) - 0.5) * 2 * error)

    def move(self, delta_time):
        self.linear_velocity += self.linear_acceleration * delta_time
        self.points += np.tile(self.linear_velocity * delta_time, (self.size, 1))


callback_called = threading.Event()
trajectory = []

def tester_callback(pose_array):
    """
    Fonction de callback de test. Elle sert principalement a synchoniser les éléments
    envoyés et reçus, afin qu'on puisse les afficher ensemble.
    @param pose_array le tableau de posistions reçu
    """
    global callback_called, trajectory

    trajectory[0] = np.append(trajectory[0], pose_array.poses[0].position.x)
    trajectory[1] = np.append(trajectory[1], pose_array.poses[0].position.y)
    trajectory[2] = np.append(trajectory[2], pose_array.poses[0].position.z)

    callback_called.set()

def storeAcceleration(imu_data, acceleration):
    imu_data.linear_acceleration.x = acceleration[0]
    imu_data.linear_acceleration.y = acceleration[1]
    imu_data.linear_acceleration.z = acceleration[2]
    return imu_data


def test_all():
    """
    Tests the feature_tracking.py code
    How do we test? The main idea is to send positions driven by a very simple model,
    but every position has a cmall error. We then get the calculated valued form the
    package, and match them with our values.
    """
    global callback_called, trajectory

    model = ArenaIntersectionModel(21, 21)
    trajectory.append(np.array([]))
    trajectory.append(np.array([]))
    trajectory.append(np.array([]))
    rospy.init_node("Tester_feature_tracking", anonymous=True)
    pub = rospy.Publisher("arena_features", PoseArray, queue_size=50)
    pub_acceleration_imu = rospy.Publisher("imu/data_raw", Imu, queue_size=50)
    sub = rospy.Subscriber("arena_static_feature_pose", PoseArray, tester_callback, queue_size=50)

    rate = rospy.Rate(60)


    x = np.zeros(model.size)
    y = np.zeros(model.size)
    z = np.zeros(model.size)
    c = np.zeros(model.size)
    
    #plt.ion()
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')

    start_time = time.time()

    model.linear_acceleration = np.array([0, 0, 0.4])

    perfect_trejectory = [np.array([]), np.array([]), np.array([])]
    for i in xrange(0, 3000):
        if i == 500:
            model.linear_acceleration = np.array([0, 0.4, 0.0])
        if i == 1000:
            model.linear_acceleration = np.array([0, 0.0, -0.4])
        if 1 == 1500:
            model.linear_acceleration = np.array([0, 0.4, 0])
        if 1 == 2000:
            model.linear_acceleration = np.array([0, 0.0, 0.0])
        if 1 == 2500:
            model.linear_acceleration = np.array([0, 0.0, -0.4])
        
        dt = time.time() - start_time
        start_time = time.time()
        
        #time.sleep(0.03)

        message = PoseArray()
        
        message.header = Header()
        message.header.stamp = rospy.Time.now()

        model.move(dt)
        points = model.get_points(0.1)

        perfect_trejectory[0] = np.append(perfect_trejectory[0], model.points[0][0])
        perfect_trejectory[1] = np.append(perfect_trejectory[1], model.points[0][1])
        perfect_trejectory[2] = np.append(perfect_trejectory[2], model.points[0][2])

        for i in xrange(0, model.size):
            pos = points[i]
            p = Pose()

            p.position.x = pos[0]
            p.position.y = pos[1]
            p.position.z = pos[2]

            x[i] = pos[0]
            y[i] = pos[1]
            z[i] = pos[2]

            if random_sample() <= 0.3:
                message.poses.append(p)
                c[i] = 0
            else:
                c[i] = 1
        
        fig.canvas.draw_idle()
        callback_called.clear()

        
        imu_data = Imu()
        imu_data.header = Header()
        imu_data.header.stamp = rospy.Time.now()
        acceleration = model.get_linear_acceleration(0.1)
        imu_data = storeAcceleration(imu_data, acceleration)

        time_start = datetime.now()


        pub.publish(message)
        ok = callback_called.wait(10)#We wait 10 sec max
        time_end = datetime.now()

        time_delta = time_end - time_start
        #print "Time elapsed = {0}".format(time_delta)

        if not ok:
            print "No message was heard"

        pub_acceleration_imu.publish(imu_data)

        if rospy.is_shutdown():
            print "Premature shutdown"
            break
    
    #ax.plot(trajectory[0], trajectory[1], trajectory[2])
    #ax.plot(perfect_trejectory[0], perfect_trejectory[1], perfect_trejectory[2])
    ax.plot(
        perfect_trejectory[0] - trajectory[0],
        perfect_trejectory[1] - trajectory[1],
        perfect_trejectory[2] - trajectory[2]
    )
    plt.show()

    if not rospy.is_shutdown():
        print "Finished sending valued, awayting ctrl-c"
        rospy.spin()


if __name__ == '__main__':
    try:
        test_all()
    except rospy.ROSInterruptException:
        pass


