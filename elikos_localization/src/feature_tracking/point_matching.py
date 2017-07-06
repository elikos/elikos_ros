#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Matches points based on the system state, and predictions of the fcu's filter.
"""
import numpy as np


def create_grid_mesh(side_points_number, side_mesure):
    u"""
    Creates a grid mesh containing side_points_number² points ane mesuring side_mesure x side_mesure
    The grid mesh is organized as follow :
    [ [x_0, y_0, 0], [x_1, y_1, 0], ..., [x_n, y_n, 0] ]
    """
    x = np.linspace(0, side_mesure, num=side_points_number)
    y = np.linspace(0, side_mesure, num=side_points_number)
    z = np.array([0])
    xs, ys, zs = np.meshgrid(x, y, z)
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1) - np.array([side_mesure/2, side_mesure/2, 0])


def closest_point(point_array, position):
    """
    Finds the closest n-dimentional point to position in point_array.
    Let n be the dimention of your points.
    :param node_array A numpy array of positions size:(x, n)
    :param position A numpy vector size:(n,)
    :return the index of the closest point in point_array
    :rtype ndarray(int)
    """
    if point_array.shape[0] is 0:
        return None
    deltas = point_array - position
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)


def match_points(input_points, points_to_match_to):
    # type: (np.ndarray, np.ndarray)->np.ndarray
    u"""
    Typical usage : for a 20mx20m arena with 21 intersections :
    match_points(detected_intersections, create_grid_mesh(21, 20))
    gives the arena points corresponding to the index of detected intersections,
    'snapped' using the closest euclidean distance, with repetitions.
    :param input_points: the points to match (detected_intersections)
    :param points_to_match_to: the points to match to (arena_intersections)
    :return: the matched points from points_to_match_to using input_points indexing
    """

    matched_points = np.empty((0, input_points.shape[-1]))

    for i in xrange(np.size(input_points, axis=0)):
        position = input_points[i]
        closest_point_index = closest_point(points_to_match_to, position)
        matched_points = np.append(matched_points, np.array([points_to_match_to[closest_point_index]]), axis=0)

    return matched_points
