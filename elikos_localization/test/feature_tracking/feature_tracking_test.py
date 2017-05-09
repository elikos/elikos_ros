#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Module de text pour le module feature_tracking d'elikos_localization
"""
import time

import rospy
from geometry_msgs.msg import PoseArray, Pose

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import numpy as np
from numpy.random import random_sample


class ArenaIntersectionModel(object):
    """ Arena model for testing. """

    def __init__(self, width, height):
        x_components = np.linspace(0, 10, width)
        y_components = np.linspace(0, 10, height)
        z_components = np.array([0])

        xv, yv, zv = np.meshgrid(x_components, y_components, z_components)
        self.points = np.reshape(np.dstack((xv, yv, zv)), (5 * 5, 3))
        self.size = width * height


    def get_points(self, error):
        return self.points + ((random_sample((self.size, 3)) - 0.5) * 2 * error )

    def move(self, value):
        self.points += np.tile(value, (self.size, 1))




def test_all():
    """
    Tests the feature_tracking.py code
    How do we test? The main idea is to send positions driven by a very simple model,
    but every position has a cmall error. We then get the calculated valued form the
    package, and match them with our values.
    """
    global x,y,z,c

    model = ArenaIntersectionModel(5, 5)
    rospy.init_node("Tester_feature_tracking", anonymous=True)
    pub = rospy.Publisher("arena_features", PoseArray, queue_size=300)

    rate = rospy.Rate(60)


    x = np.zeros(model.size)
    y = np.zeros(model.size)
    z = np.zeros(model.size)
    c = np.zeros(model.size)
    
    plt.ion()
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')

    start_time = time.time()

    for c_val in xrange(0, 300):
        dt = time.time() - start_time
        start_time = time.time()
        message = PoseArray()
        model.move(np.array([0,0,0.1]) * dt)
        points = model.get_points(0.1)
        for i in xrange(0, model.size):
            pos = points[i]
            p = Pose()
            
            p.position.x = pos[0]
            p.position.y = pos[1]
            p.position.z = pos[2]

            message.poses.append(p)
            
            x[i] = pos[0]
            y[i] = pos[1]
            z[i] = pos[2]
            c[i] = c_val
        
        ax.scatter(x, y, z, c=c, cmap=plt.hot())
        fig.canvas.draw_idle()
        pub.publish(message)
        plt.pause(1)
        ax.cla()

        if rospy.is_shutdown():
            print "Premature shutdown"
            break
    

    if not rospy.is_shutdown():
        print "Finished sending valued, awayting ctrl-c"
        rospy.spin()



test_all()


