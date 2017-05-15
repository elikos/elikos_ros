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
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

import numpy as np
from numpy.random import random_sample


class ArenaIntersectionModel(object):
    """ Arena model for testing. """

    def __init__(self, width, height, arena_size=20):
        self.size = width * height
        x_components = np.linspace(0, arena_size, width)
        y_components = np.linspace(0, arena_size, height)
        z_components = np.array([0])

        xv, yv, zv = np.meshgrid(x_components, y_components, z_components)
        self.points = np.reshape(np.dstack((xv, yv, zv)), (self.size, 3))


    def get_points(self, error):
        return self.points + ((random_sample((self.size, 3)) - 0.5) * 2 * error )

    def move(self, value):
        self.points += np.tile(value, (self.size, 1))


callback_called = threading.Event()
trajectories = []

def tester_callback(pose_array):
    """
    Fonction de callback de test. Elle sert principalement a synchoniser les éléments
    envoyés et reçus, afin qu'on puisse les afficher ensemble.
    @param pose_array le tableau de posistions reçu
    """
    global callback_called, trajectories

    for i in xrange(len(pose_array.poses)):
        trajectories[i][0] = np.append(trajectories[i][0], pose_array.poses[i].position.x)
        trajectories[i][1] = np.append(trajectories[i][1], pose_array.poses[i].position.y)
        trajectories[i][2] = np.append(trajectories[i][2], pose_array.poses[i].position.z)

    callback_called.set()

def test_all():
    """
    Tests the feature_tracking.py code
    How do we test? The main idea is to send positions driven by a very simple model,
    but every position has a cmall error. We then get the calculated valued form the
    package, and match them with our values.
    """
    global callback_called, trajectories

    model = ArenaIntersectionModel(21, 21)
    for _ in xrange(model.size):
        trajectories.append([np.array([]), np.array([]), np.array([])])
    rospy.init_node("Tester_feature_tracking", anonymous=True)
    pub = rospy.Publisher("arena_features", PoseArray, queue_size=50)
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

    for _ in xrange(0, 300):
        
        dt = time.time() - start_time
        start_time = time.time()
        
        message = PoseArray()
        
        message.header = Header()
        message.header.stamp = rospy.Time.now()

        model.move(np.array([0,0,0.1]) * dt)
        points = model.get_points(0.1)
        for i in xrange(0, model.size):
            pos = points[i]
            p = Pose()

            p.position.x = pos[0]
            p.position.y = pos[1]
            p.position.z = pos[2]

            x[i] = pos[0]
            y[i] = pos[1]
            z[i] = pos[2]

            if random_sample() <= 1:
                message.poses.append(p)
                c[i] = 0
            else:
                c[i] = 1
        
        fig.canvas.draw_idle()
        callback_called.clear()

        time_start = datetime.now()

        pub.publish(message)
        ok = callback_called.wait(10)#We wait 10 sec max
        time_end = datetime.now()

        time_delta = time_end - time_start
        print "Time elapsed = {0}".format(time_delta)

        if not ok:
            print "No message was heard"

        ax.scatter(x, y, z, c=c, cmap=plt.hot())
        for i in xrange(len(trajectories)):
            trajectory = trajectories[i]
            ax.plot(trajectory[0], trajectory[1], trajectory[2])

        plt.draw()
        plt.pause(1)
        ax.cla()

        if rospy.is_shutdown():
            print "Premature shutdown"
            break
    

    if not rospy.is_shutdown():
        print "Finished sending valued, awayting ctrl-c"
        rospy.spin()


if __name__ == '__main__':
    try:
        test_all()
    except rospy.ROSInterruptException:
        pass


