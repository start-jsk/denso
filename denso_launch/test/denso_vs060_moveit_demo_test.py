#!/usr/bin/env python

PKG = 'denso_launch'

import unittest
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from moveit_commander import MoveGroupCommander, conversions

class TestDensoMoveit(unittest.TestCase):
    arm = None
    def setUp(self):
        self.arm = MoveGroupCommander("manipulator")

    def test_moveit(self):
        p = [ 0.35, -0.35, 0.4]
        pose = PoseStamped(header = rospy.Header(stamp = rospy.Time.now(), frame_id = '/BASE'),
                           pose = Pose(position = Point(*p),
                                       orientation = Quaternion(*quaternion_from_euler(1.57, 0, 1.57, 'sxyz'))))
        self.arm.set_pose_target(pose)
        self.assertTrue(self.arm.go() or self.arm.go())

if __name__ == '__main__':
    import rostest
    rospy.init_node("denso_vs060_moveit_demo_test")
    rostest.rosrun(PKG, 'denso_vs060_moveit_demo_test', TestDensoMoveit)


