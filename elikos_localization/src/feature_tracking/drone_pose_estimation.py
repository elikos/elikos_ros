#!/usr/bin/env python
#-*- coding: utf-8 -*-u

import rospy
import tf
import elikos_ros.msg as elikos_ros#FIXME il faut fix les dépendences du package
import point_matching as matching




g_arena_points = matching.create_grid_mesh(21, 20)


def process_points(image_unperspectived_points, estimated_3d_points):
    global g_arena_points

    real_3d_points = matching.match_points(estimated_3d_points, g_arena_points)



if __name__ == '__main__':
    rospy.init_node("elikos_vision_estimator")

    points_sub = rospy.Subscriber("points_in", elikos_ros.LocalizationPoints)
    inverse_transform_sub = rospy.Subscriber("inverse_transform", elikos_ros.StampedMatrix3)

