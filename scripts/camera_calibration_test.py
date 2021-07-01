#!/usr/bin/env python
from os import read
from time import sleep
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from ros_numpy import numpify
from geometry_msgs.msg import PoseArray
import numpy as np
import cv2 as cv
from math import sqrt, fabs


ground_truth = 0.0341
chess_board_distances_y = []
chess_board_distances_x = []
for row in range(5):
    y = ground_truth * (row+1)
    chess_board_distances_y.append([y]*9)
for col in range(8):
    x = ground_truth * (col+1)
    chess_board_distances_x.append([x]*6)

ground_truth_y = np.array(chess_board_distances_y)
ground_truth_x = np.array(chess_board_distances_x).T


def distance_data(distances):
    mean_error = np.mean(distances)
    min_error = np.min(distances)
    max_error = np.max(distances)
    print(distances)
    print("min error = ", min_error)
    print("max error = ", max_error)
    print("mean error = ", mean_error)


def distance(pose1, pose2):
    return sqrt((pose2.position.x - pose1.position.x)**2 + (pose2.position.y - pose1.position.y)**2 + (pose2.position.z - pose1.position.z)**2)


def calc_dist_cols(msg):
    global ground_truth
    poses = msg.poses
    corners = np.array(poses).reshape((9, 6)).T
    distances = np.zeros((6, 8))
    for col in range(8):
        for row in range(6):
            # distances[row, col] = distance(corners[row, col], corners[row, col+1])
            distances[row, col] = distance(corners[row, 0], corners[row, col+1])

    # distances = np.fabs(distances - ground_truth)  
    print(ground_truth_x)
    print(distances)
    distances = np.fabs(distances - ground_truth_x)      
    distance_data(distances)

def calc_dist_rows(msg):
    global ground_truth
    poses = msg.poses
    corners = np.array(poses).reshape((9, 6)).T
    distances = np.zeros((5, 9))
    for col in range(9):
        for row in range(5):
            # distances[row, col] = distance(corners[row, col], corners[row+1, col])
            distances[row, col] = distance(corners[0, col], corners[row+1, col])

    # distances = np.fabs(distances - ground_truth)  
    print(ground_truth_y)
    print(distances)
    distances = np.fabs(distances - ground_truth_y)      
    distance_data(distances)
    
    
def cb(msg):
    print(len(msg.poses))
    print("Poses = ", msg.poses)
    calc_dist_rows(msg)
    calc_dist_cols(msg)
    
    
rospy.init_node("camera_calibration_test")
rospy.Subscriber("/px_to_xyz", PoseArray, cb)
rospy.spin()