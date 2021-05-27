#!/usr/bin/env python
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped
import rospy
import tf

rospy.init_node("tool_cal_test")
rospy.sleep(1)
transformer_listener = tf.TransformListener()
transformer_listener.waitForTransform("base_link", "tool0", rospy.Time(), rospy.Duration(1))
print(transformer_listener.lookupTransform("base_link", "tool0", rospy.Time()))