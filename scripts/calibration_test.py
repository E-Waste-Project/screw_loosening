#!/usr/bin/env python
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped
from std_msgs.msg import String
import rospy
import tf
from copy import deepcopy


class Irb120():

    def __init__(self, group):
        self.group = MoveGroupCommander(group)
        self.__z_offset = 0.01
        self.__x_offset = 0
        self.__y_offset = 0
        
        self.transformer_listener = tf.TransformListener()
        rospy.Subscriber("/px_to_xyz", PoseArray, self.poses_callback)
        self.trans_pub_test = rospy.Publisher("trans_pose_test", PoseArray, queue_size=1)
        self.calc_transform_pub = rospy.Publisher("calculate_transfrom_data", PoseArray, queue_size=1)
        
        self.points_poses_from_base_arr = PoseArray()
        self.cam_poses_arr = PoseArray()

    def go_to_screw(self, screw_location, quat, ref_frame="base_link"):
        self.group.set_pose_reference_frame(ref_frame) 
        # approach
        set_point = Pose()
        set_point.position.x = screw_location.position.x + self.__x_offset
        set_point.position.y = screw_location.position.y + self.__y_offset
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
        self.cam_poses_arr.header.frame_id = "calibrated_frame"
        self.cam_poses_arr.poses = deepcopy(pose_arr_msg.poses)
        print(self.cam_poses_arr)
        # trans_poses = self.__transform_poses("table_link", "camera_depth_optical_frame", pose_arr_msg)
        trans_poses = self.__transform_poses("base_link", "calibrated_frame", pose_arr_msg)
        self.trans_pub_test.publish(pose_arr_msg)
        print ("publish pose array raw")
        # raw_input()
        self.trans_pub_test.publish(trans_poses)
        print ("publish pose array transformed")
        # raw_input()
        quat_base_link = [0, 1, 0, 0]
        '''
        TODO :----------------------------
        '''
        # trans_poses = pose_arr_msg
        for i in range(len(trans_poses.poses)):
            ef_pose = self.go_to_screw(trans_poses.poses[i], quat_base_link)
            print("-------------------------")
            print("screw_location", trans_poses.poses[i].position)
            print("tool_pose = ", ef_pose.pose.position)
            print("-------------------------")
            rospy.sleep(5)
        #     print("Please Correct The error Manually")
        #     raw_input()
        #     self.transformer_listener.waitForTransform(
        #     "base_link", "link_6", rospy.Time(), rospy.Duration(1))
        #     t, r = self.transformer_listener.lookupTransform("base_link", "link_6", rospy.Time())
        #     pose = Pose()
        #     pose.position.x, pose.position.y, pose.position.z = t[0], t[1], t[2]
        #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = r[0], r[1], r[2], r[3]
        #     self.points_poses_from_base_arr.header.frame_id = "base_link"
        #     self.points_poses_from_base_arr.poses.append(pose)
        # print(self.points_poses_from_base_arr)        
            

    
if __name__=="__main__":
    rospy.init_node("loosen_it")
    Irb120("cutting_tool")
    rospy.spin()
    