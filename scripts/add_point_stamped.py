#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

rospy.init_node("add_point_stamped")
pub = rospy.Publisher("add_point_stamped", PointStamped, queue_size=1)
rospy.sleep(1)

msg = PointStamped()
msg.header.frame_id = "camera_color_optical_frame"
msg.header.stamp = rospy.Time.now()

msg.point.x = 0
msg.point.y = 0
msg.point.z = 0.5
while not rospy.is_shutdown():
    pub.publish(msg)
    rospy.sleep(0.1)

