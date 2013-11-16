#!/usr/bin/env python
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander import MoveGroupCommander, conversions
from tf.transformations import quaternion_from_euler
import rospy
import os
from subprocess import check_call
import roslib
roslib.load_manifest("denso_pendant_publisher")
roslib.load_manifest("actionlib_msgs")
import denso_pendant_publisher.msg
import std_msgs.msg
import actionlib_msgs.msg
rospy.init_node("test_vs060_moveit")

g_runnable = False

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
            rospy.loginfo("hello")
            running_pub.publish(std_msgs.msg.Bool(g_runnable))
        elif not g_prev_status.button_ok and msg.button_ok: # oked
            g_runnable = True
            running_pub.publish(std_msgs.msg.Bool(g_runnable))
    g_prev_status = msg

sub = rospy.Subscriber("/denso_pendant_publisher/status", denso_pendant_publisher.msg.PendantStatus, pendantCB)
arm = MoveGroupCommander("manipulator")
running_pub = rospy.Publisher("/irex_demo_running", std_msgs.msg.Bool);
#cancel_pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/cancel", actionlib_msgs.msg.GoalID);
cancel_pub = rospy.Publisher("/move_group/cancel", actionlib_msgs.msg.GoalID);


SCENE_FILE = os.path.join(os.path.dirname(__file__), "..", "model", "irex_model.scene")
LOAD_SCENE_PROG = os.path.join(os.path.dirname(__file__), "..", "bin", "publish_scene_from_text")

def demo() :
    # load scene
    global g_runnable
    running_pub.publish(std_msgs.msg.Bool(g_runnable))
    check_call([LOAD_SCENE_PROG, SCENE_FILE])
    for p in [[ 0.35, -0.35, 0.4],
              [ 0.6,  0.0, 0.4],
              [ 0.35,  0.35, 0.4],
              [ 0.6,  0.0, 0.2],
              [ 0.4,  0.0, 0.8]]:
        running_pub.publish(std_msgs.msg.Bool(g_runnable))
        if g_runnable:
            print "set_pose_target(", p, ")"
            pose = PoseStamped(header = rospy.Header(stamp = rospy.Time.now(), frame_id = '/BASE'),
                               pose = Pose(position = Point(*p),
                                           orientation = Quaternion(*quaternion_from_euler(1.57, 0, 1.57, 'sxyz'))))

            arm.set_pose_target(pose)
            arm.go() or arm.go() or rospy.logerr("arm.go fails")
            rospy.sleep(1)
            if rospy.is_shutdown():
                return

if __name__ == "__main__":
    while not rospy.is_shutdown():
        demo()

