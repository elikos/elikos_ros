#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Plots the difference between two tf trees using R.
"""

import math

import numpy as np
import quaternion as qt

import rospy
import tf as ros_tf
from rospy.timer import TimerEvent
from rospy.timer import TimerEvent

rospy.init_node("tf_diff_plotter", anonymous=True)

tf_child = rospy.get_param("child_frame", "elikos_vision_debug")
tf_parent = rospy.get_param("parent_frame", "elikos_vision")

tf_listener = ros_tf.TransformListener()

# x, y, z, yaw
diffs = np.empty((4,0))



def update_tf_log(time_event):
    # type:(rospy.timer.TimerEvent)->None
    global diffs, tf_child, tf_parent

    try:
        (trans, rot) = get_tf_transform(tf_parent, tf_child, time_event.current_expected, rospy.Duration(0.1))
        rot_axis = qt.as_rotation_vector(rot)
        angle = rot_axis[0] * rot_axis[0] + \
                rot_axis[1] * rot_axis[1] + \
                rot_axis[2] * rot_axis[2]
    except Exception:
        rospy.logwarn("No tf found!")
        trans = (0,0,0,0)
        angle = 0

    added = np.array([[trans[0], trans[1], trans[2], math.sqrt(angle)]]).T

    diffs = np.append(diffs, added, axis=1)

def get_tf_transform(source_frame, dest_frame, time, timeout):
    # type: (str, str, rospy.Time, rospy.Duration)->(np.ndarray, quaternion.quaternion)
    global tf_listener
    tf_listener.waitForTransform(source_frame, dest_frame, time, timeout)
    (trans, rot) = tf_listener.lookupTransform(source_frame, dest_frame, time)
    return np.array(trans), qt.quaternion(rot[3], rot[0], rot[1], rot[2])


raw_input("Press Enter to start...")
rospy.Timer(rospy.Duration(0, nsecs=10000000), update_tf_log)

try:
    raw_input("Press Enter to stop...")
except rospy.ROSInterruptException:
    pass #Caught exception, now finalising log

np.save("result", diffs)

