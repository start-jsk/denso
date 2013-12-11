#!/usr/bin/env python

import rospy
import os
import roslib
roslib.load_manifest("denso_pendant_publisher")
roslib.load_manifest("actionlib_msgs")

import denso_pendant_publisher.msg
import std_msgs.msg
import actionlib_msgs.msg

rospy.init_node("moveit_canceler")

g_runnable = True

g_prev_status = None


def pendantCB(msg):
    global g_runnable, g_prev_status
    if g_prev_status:
        if (not g_prev_status.button_cancel and msg.button_cancel) or (not g_prev_status.button_stop and msg.button_stop): # canceled or stopped
            g_runnable = False
            # here we should send cancel
            cancel = actionlib_msgs.msg.GoalID()
            cancel.id = ""
            cancel_pub.publish(cancel)
            rospy.loginfo("cancel")
    g_prev_status = msg


sub = rospy.Subscriber("/denso_pendant_publisher/status", denso_pendant_publisher.msg.PendantStatus, pendantCB)
cancel_pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/cancel", actionlib_msgs.msg.GoalID);
# cancel_pub = rospy.Publisher("/move_group/cancel", actionlib_msgs.msg.GoalID);
rospy.spin()
