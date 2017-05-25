# -*- coding: utf-8 -*-
u"""
Custom implementation of the UnscentedKalmanFiler, that allows multiple different mesurements.
"""
from filterpy.kalman import UnscentedKalmanFilter
#import numpy as np

import threading
import rospy

class SystemState(object):
    u"""
    Stores the current system state.
    """
    def __init__(self, time, z, R, x, P, ukf, filtering_function):
        self.time = time
        self.z = z
        self.R = R
        self.x = x
        self.P = P
        self.ukf = ukf
        self.filtering_function = filtering_function
    
    def calculate(self, dt, Q):
        """
        Calculates, using the internal ukf, the new x and P and returns those.
        """
        self.ukf.x = self.x
        self.ukf.P = self.P
        self.ukf.Q = Q
        self.filtering_function(self.ukf, self.z, self.R, dt)
        return self.ukf.x, self.ukf.P

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
        self.lower_bound = self.items[-2].time
        return self.items.pop()


    def add_item(self, system_state):
        """
        Adds an item to the list. Returns the index of the item, or None if it didn't fit. Also returns the item (if any) that was popped from the list.
        """
        if self.current_count < self.max_count:
            return self.__insert(system_state), None
        elif self.lower_bound < system_state.time:
            itm = self.__remove_last()
            return self.__insert(system_state), itm
        else:
            return None, None #Could not insert

def default_filtering_function(ukf, z, R, dt):
    ukf.predict(dt=dt)
    ukf.update(z=z, R=R)

class MultiUnscentedKalmanFilter(object):
    u"""
    Class that manages multiple unscented kalman filters.
    """

    def __init__(self, initial_x, initial_P, Q_generator, Q_generator_args=(), message_queue_size=5):
        self.filters={}
        self.x = initial_x
        self.P = initial_P
        self.Q_generator = Q_generator
        self.Q_generator_args = Q_generator_args
        self.active = None
        self.message_queue = SystemStateList(message_queue_size)
        self.lock = threading.Lock()

    def calculate_for_new_message(self, z, R, time, filter_key, filtering_function=default_filtering_function):
        """
        Calculates the whole filters state based on the new message
        Parameters:
        -----------
            z : the mesurement vector
            R : the mesurement vector co-variance
            time : the time (NOT delta-time) of the mesurement
            filter_key : the key (string) of the filter to use
            filtering_function : an optional function to handle super-special cases. Refer to the default filtering function.
        """
        message = SystemState(time, z, R, None, None, self.filters[filter_key], filtering_function)

        self.lock.acquire()

        message_position, removed_message = self.message_queue.add_item(message)

        if message_position is None:
            rospy.logdebug_throttle(30, "One or more messages were dropped.")

        elif message_position is 0:
            message.x = self.x
            message.P = self.P

            dt = 0
            if self.message_queue.current_count > 1:
                dt = message.time - self.message_queue.items[1].time

            self.x, self.P = message.calculate(dt, self.Q_generator(dt, *self.Q_generator_args))

        else:
            next_message = self.message_queue.items[message_position - 1]
            message.x = next_message.x
            message.P = next_message.P

            #setup delta-time
            dt = 0
            if message_position < len(self.message_queue.items) - 1:
                dt = message.time - self.message_queue.items[message_position + 1].time
            else:#the message is the last
                if removed_message is not None:
                    dt = message.time - removed_message.time
                else:#the message is the last message ever
                    dt = 0

            last_message = message
            for current_message in self.message_queue.items[(message_position - 1)::-1]:

                current_message.x, current_message.P = last_message.calculate(dt, self.Q_generator(dt, *self.Q_generator_args))

                #for next loop
                dt = current_message.time - last_message.time
                last_message = current_message

            self.x, self.P = last_message.calculate(dt, self.Q_generator(dt, *self.Q_generator_args))

        self.lock.release()



