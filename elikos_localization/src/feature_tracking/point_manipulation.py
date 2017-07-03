#!/usr/bin/env python
#-*- coding: utf-8 -*-u
u"""
Manipulate points in 3d using transformation matrices, quaternions,
rotation matrices, projection matrices and more.
"""
import numpy as np
import quaternion as qt


def make_homogeneous(points):
    u"""
    Makes points homogeneous. Suppose
     [[0, 1], [0, 2]]
    :param points: an ndarray of size (x, n) for x points of n dimensions
    :return: an ndarray of size (x, n+1) of homogeneous coordinates
    """
    points_number = np.size(points, axis=0)
    return np.append(points, np.expand_dims(np.ones((points_number,)), axis=1), axis=1)


def normalize_points(homogeneous_points):
    ws = np.expand_dims(homogeneous_points[..., -1], axis=1)
    return np.true_divide(homogeneous_points, ws)


def transform_points(matrix, points):
    # type: (np.ndarray, np.ndarray) -> np.ndarray
    homogeneous_points = make_homogeneous(points)
    transformed_points = np.matmul(matrix, homogeneous_points.T).T
    return normalize_points(transformed_points)[...,0:-1]


def transform_points_simple(points, translation, rotation):
    tranlation_matrix = create_translation_matrix(translation)
    rotation_matrix = create_rotation_matrix(rotation)
    return transform_points(
        np.matmul(tranlation_matrix, rotation_matrix),
        points
    )

def create_translation_matrix(translation):
    return np.array([
        [1, 0, 0, translation[0]],
        [0, 1, 0, translation[1]],
        [0, 0, 1, translation[2]],
        [0, 0, 0, 1]
    ])


def create_rotation_matrix(quaterion):
    # type: (qt.quaterion)->np.ndarray
    mat = np.identity(4, dtype=np.float)
    mat[0:3, 0:3] = qt.as_rotation_matrix(quaterion)
    return mat


def create_quaterion_from_tf(tf_quaterion):
    # type: (tuple)->qt.quaternion
    return qt.quaternion(tf_quaterion[3], tf_quaterion[0], tf_quaterion[1], tf_quaterion[2])


def create_tf_from_quaterion(quaterion):
    # type: (qt.quaternion)->tuple
    return (quaterion.x, quaterion.y, quaterion.z, quaterion.w)


def tf_to_matrix(tf_translation, tf_quaterion):
    # type: (tuple, tuple)->np.ndarray
    tranlation_matrix = create_translation_matrix(tf_translation)
    rotation_matrix = create_rotation_matrix(qt.quaternion(tf_quaterion[3], tf_quaterion[0], tf_quaterion[1], tf_quaterion[2]))
    return np.matmul(tranlation_matrix, rotation_matrix)


def create_3d_projection_matrix(plane):
    # type: (float)->np.ndarray
    u"""
    Creates a projection matrix to project points on the xy plane at 'plane' distance of the z axis
    :param plane: the distance of the z axis to project onto
    :return: the projection matrix
    """
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, np.true_divide(1, plane), 0]
    ])





##########
#
# OpenCV utils
#
##########

def prepare_points_for_cv(input_points):
    # type: (np.ndarray)->np.ndarray
    u"""
    Takes an array of shape (x, n), of x n-dimentional points,
     and transforms it into an array of shape (x, 1, n), the
     shape that is used by openCV. If the points are floats,
     makes them float32 (again, this is for openCv to work)
    :param input_points: an array of shape (x, n)
    :return: an array of shape (x, 1, n)
    """
    if input_points.dtype == np.float:
        input_points = input_points.astype(np.float32)
    return np.ascontiguousarray(input_points).reshape(
        (input_points.shape[0], 1, input_points.shape[-1])
    )

def unconvert_points_from_cv(input_points):
    # type: (np.ndarray)->np.ndarray
    u"""
    Opposite of prepare_points_for_cv
    :param input_points: an array of shape (x, 1, n)
    :return: an array of shape (x, n)
    """
    return input_points.reshape(
        (input_points.shape[0], input_points.shape[-1])
    )

