#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Utility class that allows access of ground thuth position from gazebo.
"""
import itertools

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates


class GroundTruth(object):
    """
    Class that records the grouns truth position of an object in a gazebo simulation.
    """
    def __init__(self, model_name="iris"):
        rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            callback=_ground_truth_quad_position,
            callback_args=(self, model_name),
            queue_size=1
        )
        self.position = np.zeros((3,))

def _ground_truth_quad_position(message, args):
    owner, quad_name = args
    for name, pose in itertools.izip(message.name, message.pose):
        if name == quad_name:
            owner.position[0] = pose.position.x
            owner.position[1] = pose.position.y
            owner.position[2] = pose.position.z

