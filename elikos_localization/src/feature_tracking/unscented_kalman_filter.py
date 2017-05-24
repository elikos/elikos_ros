# -*- coding: utf-8 -*-
u"""
Custom implementation of the UnscentedKalmanFiler, that allows multiple different mesurements.
"""
from filterpy.kalman import UnscentedKalmanFilter
#import numpy as np

class MultiUnscentedKalmanFilter(object):
    u"""
    Class that manages multiple unscented kalman filters.
    """

    def __init__(self, initial_x, initial_P):
        self.filters={}
        self.x = initial_x
        self.P = initial_P
        self.active = None

    def set_active(self, key):
        if self.active is not None:
            self.x = self.active.x
            self.P = self.active.P
        self.active = self.filters[key]
        self.active.x = self.x
        self.active.P = self.P
    
    def update(self, z, R=None, UT=None, hx_args=()):
        self.active.update(z, R=R, UT=UT, hx_args=hx_args)

