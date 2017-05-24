# -*- coding: utf-8 -*-
u"""
Custom implementation of the UnscentedKalmanFiler, that allows multiple different mesurements.
"""
from filterpy.kalman import UnscentedKalmanFilter
#import numpy as np

import bisect

class SystemState(object):
    u"""
    Stores the current system state.
    """
    def __init__(self, time, z, R, x, P, filter_key, filtering_function):
        self.time = time
        self.z = z
        self.R = R
        self.x = x
        self.P = P
        self.filter_key = filter_key
        self.filtering_function = filtering_function


class SystemStateList(object):
    def __init__(self, max_count):
        self.max_count = max_count
        self.current_count = 0
        self.items = []
        self.lower_bound = 0

    def __insert(self, system_state):
        """
        Inserts element in order and returns the index of the inserted element
        """
        for i, ith_system_state in enumerate(self.items):
            if system_state.time >= ith_system_state.time:
                self.items.insert(i, system_state)
                return i

        self.items.append(system_state)
        self.lower_bound = system_state.time
        return len(self.items) - 1

    def __remove_last(self):
        """
        Removes last element, keeps lower bound up to date.
        Do not call if less that 2 elements, or else this will crash.
        """
        self.items.pop()
        self.lower_bound = self.items[-1].time


    def add_item(self, system_state):
        """
        Adds an item to the list. Returns the index of the item, or None if it didn't fit.
        """
        if self.current_count < self.max_count:
            return self.__insert(system_state)
        elif self.lower_bound < system_state.time:
            self.__remove_last()
            return self.__insert(system_state)
        else:
            return None #Could not insert

def default_filtering_function(ukf, z, R, dt):
    ukf.predict(dt=dt)
    ukf.update(z=z, R=R)

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

    def calculate_for_new_message(self, z, R, time, filter_key, filternig_function):
        """
        Calculates the whole filters state based on the new message
        Parameters:
        -----------
        z : the mesurement vector
        R : the mesurement vector co-variance
        time : the time (NOT delta-time) of the mesurement
        filter_key : the key (string) of the filter to use
        filtering_function : an optional function to handle super-special cases. Refer to the
        default filtering function.
        """
        pass#TODO

