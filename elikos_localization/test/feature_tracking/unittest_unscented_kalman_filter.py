#!usr/bin/env python
PKG = 'elikos_localization'

import sys
import unittest

import numpy as np
from feature_tracking import unscented_kalman_filter

class TestSystemStateList(unittest.TestCase):

    def test_add_item_bb(self):
        item = unscented_kalman_filter.SystemStateList(3)

        test1 = unscented_kalman_filter.SystemState(10, None, None, None, None, None, None)
        test2 = unscented_kalman_filter.SystemState(11, None, None, None, None, None, None)
        test3 = unscented_kalman_filter.SystemState(9, None, None, None, None, None, None)
        test4 = unscented_kalman_filter.SystemState(10, None, None, None, None, None, None)
        test5 = unscented_kalman_filter.SystemState(8, None, None, None, None, None, None)

        i, it = item.add_item(test1)
        self.assertIsNone(it)
        self.assertEqual(i, 0)

        i, it = item.add_item(test2)
        self.assertIsNone(it)
        self.assertEqual(i, 0)

        i, it = item.add_item(test3)
        self.assertIsNone(it)
        self.assertEqual(i, 2)

        i, it = item.add_item(test4)
        self.assertIs(it, test3)
        self.assertEqual(i, 1)

        i, it = item.add_item(test5)
        self.assertIs(it, test5)
        self.assertIsNone(i)



if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_some_feature', TestSystemStateList)
