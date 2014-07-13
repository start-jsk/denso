#!/usr/bin/env python

PKG = 'denso_launch'

import unittest

class TestDensoMoveit(unittest.TestCase):
    def test(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'denso_vs060_moveit_demo_test', TestDensoMoveit)


