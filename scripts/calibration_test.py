#!/usr/bin/env python
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped
from std_msgs.msg import String
import rospy
import tf


class Irb120():

    def __init__(self, group):
        self.group = MoveGroupCommander(group)
        self.__z_offset = 0.01
        
        self.transformer_listener = tf.TransformListener()
        rospy.Subscriber("/cut_xyz", PoseArray, self.poses_callback)
        self.trans_pub_test = rospy.Publisher("trans_pose_test", PoseArray, queue_size=1)

    def go_to_screw(self, screw_location, quat, ref_frame="base_link"):
        self.group.set_pose_reference_frame(ref_frame) 
        # approach
        set_point = Pose()
        set_point.position.x = screw_location.position.x
        set_point.position.y = screw_location.position.y
        set_point.position.z = screw_location.position.z + self.__z_offset

        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        self.group.clear_pose_targets()
        self.group.set_pose_target(set_point)
        approach_result = self.group.go()
        return self.group.get_current_pose(end_effector_link=self.group.get_end_effector_link())
        
    def __transform_poses(self, target_frame, source_frame, pose_arr):
        print ("here")
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(target_frame, source_frame, rospy.Time(),rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)
            
        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()    
        return trans_pose_arr
               
    def poses_callback(self, pose_arr_msg):
        # trans_poses = self.__transform_poses("table_link", "camera_depth_optical_frame", pose_arr_msg)
        trans_poses = self.__transform_poses("base_link", "calibrated_frame", pose_arr_msg)
        self.trans_pub_test.publish(pose_arr_msg)
        print ("publish pose array raw")
        # raw_input()
        self.trans_pub_test.publish(trans_poses)
        print ("publish pose array transformed")
        # raw_input()2
        quat_base_link = [1, 0, 0, 0]
        for i in range(len(trans_poses.poses)):
            ef_pose = self.go_to_screw(trans_poses.poses[i], quat_base_link)
            print("-------------------------")
            print("screw_location", trans_poses.poses[i].position)
            print("tool_pose = ", ef_pose.pose.position)
            print("-------------------------")
            rospy.sleep(5)
            # raw_input()

    
if __name__=="__main__":
    rospy.init_node("loosen_it")
    Irb120("tool_calibration_group")
    rospy.spin()
    