#!/usr/bin/env python
#-*- coding: utf-8 -*-u

import numpy as np

import rospy
import tf
import message_filters

import elikos_main.msg as elikos_main

import point_matching as matching
import message_interface as msgs





g_arena_points = matching.create_grid_mesh(21, 20)








def process_points(image_unperspectived_points, estimated_3d_points):
    global g_arena_points

    real_3d_points = matching.match_points(estimated_3d_points, g_arena_points)





def message_callback(localization_points, inverse_transform):
    # type: (elikos_main.IntersectionArray, elikos_main.StampedMatrix3)->None
    points_image, points_arena = msgs.deserialize_intersections(localization_points)
    print points_image
    print points_arena

    




if __name__ == '__main__':
    rospy.init_node("elikos_vision_estimator")

    points_sub = message_filters.Subscriber("points_in", elikos_main.IntersectionArray)
    inverse_transform_sub = message_filters.Subscriber("inverse_transform", elikos_main.StampedMatrix3)

    synchroniser = message_filters.TimeSynchronizer([points_sub, inverse_transform_sub], 50)

    synchroniser.registerCallback(message_callback)
    rospy.loginfo("Initialization finished. Now tracking!")
    rospy.spin()