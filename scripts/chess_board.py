#!/usr/bin/env python
from os import read
from time import sleep
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from ros_numpy import numpify

import numpy as np
import cv2 as cv


class Calibrate:
    def __init__(self, read_img=False, publish_topic='/cutting_path'):
        self.image = None
        self.path_publisher = rospy.Publisher(publish_topic,
                                         Float32MultiArray,
                                         queue_size=1)
        self.read_img = read_img
        self.image_topic = "/camera/color/image_raw"

    def recieve_img(self):
        if self.read_img:
            self.image = cv.imread("/home/zaferpc/Pictures/pattern.png")
            self.image = cv.resize(self.image, (1280, 720))
        else:
            # # Wait for rgb camera stream to publish a frame.e
            print("Waiting for image")
            image_msg = rospy.wait_for_message(self.image_topic, Image)
            print("Image Received")
            # Convert msg to numpy image.
            self.image = numpify(image_msg)
    
    def detect_corners(self, size=(7, 6)):
        img = self.image.copy()
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        cv.imshow('img', img)
        cv.waitKey(0)
        cv.destroyAllWindows()
        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Arrays to store object points and image points from all the images.
        gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, size, None)
        # If found, add object points, image points (after refining them)
        print(ret)
        if ret == True:
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # Draw and display the corners
            # cv.drawChessboardCorners(img, size, corners2, ret)
            corner_points = []
            for i in range(corners2.shape[0]):
                # cv.circle(img, tuple(corners2[i][0]), 5, (0, 255, 0), 3)
                # cv.imshow('img', img)
                # cv.waitKey(0)
                corner_points.append(tuple(corners2[i][0]))
            print("corners published")
            self.publish_points(corner_points)
            rospy.sleep(1)

    def publish_points(self, points):
        # Publish Cutting Path.
        path_msg = Float32MultiArray()
        for x, y in points:
            path_msg.data.append(y)
            path_msg.data.append(x)
        self.path_publisher.publish(path_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node("calibration_test")
    sleep(1)
    calib = Calibrate(read_img=False)
    while not rospy.is_shutdown():
        calib.recieve_img()
        calib.detect_corners(size=(6, 9))
        print("Input Any Key to Continue")
        raw_input()
    cv.destroyAllWindows()