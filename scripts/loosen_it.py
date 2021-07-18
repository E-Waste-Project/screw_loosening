#!/usr/bin/env python
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray, Pose, PointStamped, PoseStamped
import rospy
import tf
from std_msgs.msg import String
from abb_robot_msgs.msg import SystemState
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from cut_contour.robot_helpers import MotionServices, TransformServices, generate_spiral
from copy import deepcopy


class loosenIt():
    TOOL_GROUP = "cutting_tool"
    CAMERA_GROUP = "camera"

    def __init__(self):
        self.milling_ms = MotionServices(self.TOOL_GROUP)
        self.group = MoveGroupCommander(self.TOOL_GROUP)
        self.camera = MoveGroupCommander(self.CAMERA_GROUP)
        self.depth_of_cut = -0.0035
        self.approach_dist = 0.02
        self.retreat_dist = 0.02
        self.working_flag = False
        # self. retreat_dist =
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        rospy.Subscriber("/rws/system_states", SystemState, self.states_cb)
        self.transformer_listener = tf.TransformListener()
        rospy.Subscriber("/screw_xyz", PoseArray, self.poses_callback)
        self.trans_pub_test = rospy.Publisher(
            "trans_pose_test", PoseArray, queue_size=1)
        self.tool_quat_base_link = [0, 0.707, 0, 0.707]
        self.tool_quat_table_link = [0.707, -0.707, 0.000, -0.000]
        self.cutting_tool = rospy.ServiceProxy(
            "/rws/set_io_signal", SetIOSignal)
        self.tf_services = TransformServices()
        # self.camera.set_pose_reference_frame("base_link")
        # self.camera.set_pose_target([0.310, 0.046, 0.508, 0.000, 0.707, 0.000, 0.707], end_effector_link="camera_link")
        # self.camera.go()
        
        self.screw_offset = 0.01

    def states_cb(self, state_msg):
        if state_msg.motors_on == False and self.working_flag == True:
            failed_msg = String()
            failed_msg.data = "screw failed"
            self.done_pub.publish(failed_msg)
    
    def change_tool_status(self, status=0):
        self.cutting_tool.wait_for_service()
        cutting_tool_status = SetIOSignalRequest()
        cutting_tool_status.signal = "CuttingTool"
        cutting_tool_status.value = str(status)
        response = self.cutting_tool(cutting_tool_status)
        return response

    def approch_screw(self, screw_location, ref_frame="base_link"):

        self.group.set_pose_reference_frame(ref_frame)
        # approach
        set_point = Pose()
        set_point.position.x = screw_location.position.x
        set_point.position.y = screw_location.position.y
        set_point.position.z = screw_location.position.z + self.approach_dist

        quat = self.tool_quat_base_link

        if ref_frame == "table_link":
            quat = self.tool_quat_table_link

        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        self.group.clear_pose_targets()
        self.group.set_pose_target(set_point)
        approach_result = self.group.go()
        return approach_result

    def move_z_straight(self, screw_location, dist, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
        # set the pose_arr reference frame
        self.group.set_pose_reference_frame(ref_frame)

        # compute the plan
        goal_pose_arr = []
        set_point = Pose()
        set_point.position.x = screw_location.position.x
        set_point.position.y = screw_location.position.y
        set_point.position.z = screw_location.position.z + dist
        quat = self.tool_quat_base_link
        if ref_frame == "table_link":
            quat = self.tool_quat_table_link
        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        goal_pose_arr.append(set_point)
        plan, fraction = self.group.compute_cartesian_path(
            goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan
        filtered_plan = self.__filter_plan(plan)
        # filtered_plan = plan

        # execute the filtered plan
        final_traj = self.group.retime_trajectory(
            self.group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        result = self.group.execute(final_traj)
        return result

    def cut_sequence(self, screw_location, ref_frame="base_link", vel_scale=1, acc_scale=1):
        self.group.set_pose_reference_frame(ref_frame)

        # approach
        approach_result = self.approch_screw(screw_location)
        print("screw approach = ", approach_result)

        # cut screw
        print("Press enter to cut")
        # raw_input()

        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.poses.append(deepcopy(screw_location))
        pose_array.poses[0].position.z -= 0.05
        # pose_array.poses[0].orientation.x = 0
        # pose_array.poses[0].orientation.y = 1
        # pose_array.poses[0].orientation.z = 0
        # pose_array.poses[0].orientation.w = 0
        cut_result = self.milling_ms.move_to_touch(poses=pose_array, axis='x', force_thresh=2,
                                                   vel_scale=0.05, acc_scale=0.05)
        
        screw_pose_touched = self.tf_services.lookup_transform('base_link', 'milling_tool')
        screw_pose_touched.position.z += self.screw_offset
        rospy.sleep(1)
        
        screw_pose_touched_list = PoseArray()
        screw_pose_touched_list.poses.append(screw_pose_touched)
        
        self.milling_ms.move_straight(screw_pose_touched_list)
        rospy.sleep(1)
        
        self.milling_ms.change_tool_status('CuttingTool', status = 1)
        
        screw_pose_touched_list.poses[0].position.z -= (self.depth_of_cut + self.screw_offset)
        
        spiral_array = generate_spiral(0.005, 0.005, screw_pose_touched_list.poses[0], ref_frame = 'base_link')
        
        self.milling_ms.move_straight(spiral_array)        
        
        print("screw cutting = ", cut_result)

        # retreat
        retreat_result = self.move_z_straight(
            screw_location, ref_frame=ref_frame, dist=self.retreat_dist)
        print("screw retreat = ", retreat_result)
        # self.change_tool_status(status=0)
        
        # self.change_tool_status(status=1)
        # cut_result = self.move_z_straight(
        # screw_location, dist=self.__depth_of_cut, ref_frame=ref_frame, vel_scale=vel_scale, acc_scale=acc_scale)

        final_result = approach_result and cut_result and retreat_result
        return final_result

    def __transform_poses(self, target_frame, source_frame, pose_arr):
        print("here")
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(), rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(
                target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)

        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()
        return trans_pose_arr

    def __filter_plan(self, plan):
        last_time_step = plan.joint_trajectory.points[0].time_from_start.to_sec
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header = plan.joint_trajectory.header
        new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
        new_plan.joint_trajectory.points.append(
            plan.joint_trajectory.points[0])
        for i in range(1, len(plan.joint_trajectory.points)):
            point = plan.joint_trajectory.points[i]
            if point.time_from_start.to_sec > last_time_step:
                new_plan.joint_trajectory.points.append(point)
            last_time_step = point.time_from_start.to_sec
        return new_plan

    def poses_callback(self, pose_arr_msg):
        self.milling_ms.change_tool_status('Clamp_Off', status = 0)
        rospy.sleep(1)
        self.milling_ms.change_tool_status('Clamp_On', status = 1)
        rospy.sleep(1)
        self.working_flag = True
        trans_poses = self.__transform_poses(
            "base_link", "calibrated_frame", pose_arr_msg)
        self.trans_pub_test.publish(pose_arr_msg)
        print (trans_poses)
        for i in range(len(trans_poses.poses)):
            print("screw location = ", trans_poses.poses[i])
            result = self.cut_sequence(
                trans_poses.poses[i], vel_scale=0.001, acc_scale=0.001)
            print("screw", i, "succeeded ? ", result)
            # raw_input()
        self.working_flag = False 
        done_msg = String()
        done_msg.data = "Done"
        self.done_pub.publish(done_msg)


if __name__ == "__main__":
    rospy.init_node("loosen_it")
    loosenIt()
    rospy.spin()
