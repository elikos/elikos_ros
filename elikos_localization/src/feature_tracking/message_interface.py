#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Interface to serialize en deserealize ros messages.
"""
import numpy as np
import elikos_main.msg as elikos_main


def deserialize_intersections(localization_points):
    # type: (elikos_main.IntersectionArray)->(np.ndarray,np.ndarray)
    u"""
    Prens un message de localisation et retourne un message de points.
    :param localization_points: le message
    :return: un tuple d'un tableau des intersection en 2d et 3d
    """
    intersection_number = len(localization_points.intersections)
    array_2d_pts = np.empty((intersection_number,2))
    array_3d_pts = np.empty((intersection_number,3))
    for i, intersection in enumerate(localization_points.intersections):
        array_2d_pts[i] = (intersection.imagePosition.x, intersection.imagePosition.y)
        array_3d_pts[i] = (intersection.arenaPosition.x, intersection.arenaPosition.y, intersection.arenaPosition.z)

    return array_2d_pts, array_3d_pts



