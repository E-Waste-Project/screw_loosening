#!/usr/bin/env python
import sys
import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix
sys.path.append('/home/zaferpc/OpenSfM/')
from opensfm import transformations as tp
import yaml
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
import numpy as np

class TransformCalulator:
    
    def __init__(self):
        rospy.Subscriber("/px_to_xyz", PoseArray, self.calcTransform)
        # self.pub = rospy.Publisher("/px_to_xyz", PoseArray, queue_size=1)
        rospy.sleep(2)
        self.transformer_listener = tf.TransformListener()
        self.transformer_broadcaster = tf2_ros.StaticTransformBroadcaster()
        with open("/home/zaferpc/abb_ws/src/abb_experimental/abb_irb120_moveit_config/launch/ground_truth.yaml") as f:
            self.ground_truth = yaml.load(f)
        ground_truth_list = [[], [], []]
        for i, pose in enumerate(self.ground_truth['poses']):
            ground_truth_list[0].append(pose['position']['x'])
            ground_truth_list[1].append(pose['position']['y'])
            ground_truth_list[2].append(pose['position']['z'])
        
        self.ground_truth_arr = np.array(ground_truth_list)             
        # print(len(pose_array['poses']))
        # print(pose_array['poses'])
        # milling to link_6 transform

        # self.pub.publish(transformed_pose_array)
    def calcTransform(self, detected_msg):
        detected_list = [[], [], []]
        for i, pose in enumerate(detected_msg.poses):
            detected_list[0].append(pose.position.x)
            detected_list[1].append(pose.position.y)
            detected_list[2].append(pose.position.z)
        detected_arr = np.array(detected_list)
        
        transform = tp.affine_matrix_from_points(detected_arr, self.ground_truth_arr, False, False, usesvd=False)
        translation = translation_from_matrix(transform)
        orientation = quaternion_from_matrix(transform)
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "base_link"
        static_transformStamped.child_frame_id = "calibrated_frame"

        static_transformStamped.transform.translation.x = translation[0]
        static_transformStamped.transform.translation.y = translation[1]
        static_transformStamped.transform.translation.z = translation[2]

        static_transformStamped.transform.rotation.x = orientation[0]
        static_transformStamped.transform.rotation.y = orientation[1]
        static_transformStamped.transform.rotation.z = orientation[2]
        static_transformStamped.transform.rotation.w = orientation[3]
        self.transformer_broadcaster.sendTransform(static_transformStamped)
        self.transformer_listener.waitForTransform(
            "link_6", "calibrated_frame",  rospy.Time(), rospy.Duration(1))
        t, r = self.transformer_listener.lookupTransform(
            "link_6", "calibrated_frame", rospy.Time())
        print(t)
        print(r)

    
    def convert_function(self, pose_array):
        self.transformer_listener.waitForTransform(
            "link_6", "milling_tool",  rospy.Time(), rospy.Duration(1))
        t, r = self.transformer_listener.lookupTransform(
            "link_6", "milling_tool", rospy.Time())
        T_m_6 = quaternion_matrix(r)
        T_m_6[:-1, -1] = t
        transformed_pose_array = PoseArray()
        for pose in pose_array['poses']:
            x = pose['position']['x']
            y = pose['position']['y']
            z = pose['position']['z']
            qx = pose['orientation']['x']
            qy = pose['orientation']['y']
            qz = pose['orientation']['z']
            qw = pose['orientation']['w']
            T_6_b = quaternion_matrix([qx, qy, qz, qw])
            T_6_b[:-1, -1] = [x, y, z]
            trans_arr = T_6_b.dot(T_m_6)
            quat = quaternion_from_matrix(trans_arr)
            translation = translation_from_matrix(trans_arr)
            new_pose = Pose()
            new_pose.position.x = translation[0]
            new_pose.position.y = translation[1]
            new_pose.position.z = translation[2]
            new_pose.orientation.x = quat[0]
            new_pose.orientation.y = quat[1]
            new_pose.orientation.z = quat[2]
            new_pose.orientation.w = quat[3]
            transformed_pose_array.poses.append(new_pose)
            print (transformed_pose_array)
    
if __name__ == "__main__":
    rospy.init_node("transform_calculator")
    TransformCalulator()
    rospy.spin()