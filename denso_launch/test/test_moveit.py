#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, University of Tokyo and
# TORK (Tokyo Opensource Robotics Kyokai Association) All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Tokyo and 
#    Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac I.Y. Saito

import unittest

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from moveit_commander import MoveGroupCommander, MoveItCommanderException, RobotCommander
from moveit_msgs.msg import RobotTrajectory, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rospy


class TestMoveit(unittest.TestCase):
    _MOVEGROUP_MAIN = 'manipulator'
    _KINEMATICSOLVER_SAFE = 'RRTConnectkConfigDefault'

    @classmethod
    def setUpClass(self):
        rospy.init_node('test_moveit_vs060')
        self.robot = RobotCommander()
        self._mvgroup = MoveGroupCommander(self._MOVEGROUP_MAIN)
        # Temporary workaround of planner's issue similar to https://github.com/tork-a/rtmros_nextage/issues/170
        self._mvgroup.set_planner_id(self._KINEMATICSOLVER_SAFE)

        self._movegroups = sorted(['manipulator', 'manipulator_flange'])
        self._joints_movegroup_main = sorted(['j1', 'j2', 'j3', 'j4', 'j5', 'flange'])

    @classmethod
    def tearDownClass(self):
        True  # TODO impl something meaningful

    def _set_sample_pose(self):
        '''
        @return: Pose() with some values populated in.
        '''
        pose_target = Pose()
        pose_target.orientation.x = 0.00
        pose_target.orientation.y = 0.00
        pose_target.orientation.z = -0.20
        pose_target.orientation.w = 0.98
        pose_target.position.x = 0.18
        pose_target.position.y = 0.18
        pose_target.position.z = 0.94
        return pose_target

    def _plan(self):
        '''
        Run `clear_pose_targets` at the beginning.
        @rtype: RobotTrajectory http://docs.ros.org/api/moveit_msgs/html/msg/RobotTrajectory.html
        '''
        self._mvgroup.clear_pose_targets()

        pose_target = self._set_sample_pose()
        self._mvgroup.set_pose_target(pose_target)
        plan = self._mvgroup.plan()
        rospy.loginfo('  plan: '.format(plan))
        return plan

    def test_list_movegroups(self):
        '''Check if the defined move groups are loaded.'''
        self.assertEqual(self._movegroups, sorted(self.robot.get_group_names()))

    def test_list_activejoints(self):
        '''Check if the defined joints in a move group are loaded.'''
        self.assertEqual(self._joints_movegroup_main, sorted(self._mvgroup.get_active_joints()))

    def test_plan(self):
        '''Evaluate plan (RobotTrajectory)'''
        plan = self._plan()
        # TODO Better way to check the plan is valid.
        # Currently the following checks if the number of waypoints is not zero,
        # which (hopefully) indicates that a path is computed.
        self.assertNotEqual(0, plan.joint_trajectory.points)

    def test_planandexecute(self):
        '''
        Evaluate Plan and Execute works.
        Currently no value checking is done (, which is better to be implemented)
        '''
        self._plan()
        # TODO Better way to check the plan is valid.
        # The following checks if plan execution was successful or not.
        self.assertTrue(self._mvgroup.go())

    def test_set_pose_target(self):
        '''
        Check for simple planning, originally testd in moved from denso_vs060_moveit_demo_test.py
        '''
        self._plan()
        p = [ 0.1, -0.35, 0.4]
        pose = PoseStamped(header = rospy.Header(stamp = rospy.Time.now(), frame_id = '/BASE'),
                           pose = Pose(position = Point(*p),
                                       orientation = Quaternion(*quaternion_from_euler(1.57, 0, 1.57, 'sxyz'))))
        self._mvgroup.set_pose_target(pose)
        self.assertTrue(self._mvgroup.go() or self._mvgroup.go())

    def test_planning_scene(self):
        '''
        Check if publish_simple_scene.py is working
        '''
        rospy.wait_for_service('/get_planning_scene', timeout=20);
        get_planning_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        collision_objects = []
        # wait for 10 sec
        time_start = rospy.Time.now()
        while not collision_objects and (rospy.Time.now() - time_start).to_sec() < 10.0:
            world = get_planning_scene(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES)).scene.world
            collision_objects = world.collision_objects
            rospy.loginfo("collision_objects = " + str(world.collision_objects))
            rospy.sleep(1)
        self.assertTrue(world.collision_objects != [], world)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('vs060', 'test_moveit_vs060', TestMoveit) 
