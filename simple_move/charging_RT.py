#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
import numpy.linalg as la
import math
import time
import tf2_ros
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import Float32MultiArray

from dynamic_reconfigure.server import Server
from random import sample
import tf.transformations as tft

from std_msgs.msg import Bool


class Robot_loc:
    """机器人运动定位类
    """

    def __init__(self):
        self.x = float("inf")
        self.y = float("inf")
        self.theta = float("inf")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def ok(self):
        if self.x == float("inf") or self.y == float("inf") or self.theta == float("inf"):
            return False
        else:
            return True

    def update_loc(self, track_frame):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(track_frame, 'base_footprint', rospy.Time())
                self.x = trans.transform.translation.x
                self.y = trans.transform.translation.y
                quat_ = [trans.transform.rotation.x, trans.transform.rotation.y,
                         trans.transform.rotation.z, trans.transform.rotation.w]
                self.theta = tft.euler_from_quaternion(quat_)[2]
                return self
                break
                # self.output()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()

    def update_loc_2(self, track_frame, source_frame):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(track_frame, source_frame, rospy.Time())
                self.x = trans.transform.translation.x
                self.y = trans.transform.translation.y
                quat_ = [trans.transform.rotation.x, trans.transform.rotation.y,
                         trans.transform.rotation.z, trans.transform.rotation.w]
                self.theta = tft.euler_from_quaternion(quat_)[2]
                return self
                break
                # self.output()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()

    def output(self):
        print "loc_x: " + str(self.x) + "---- loc_y: " + str(self.y) + "---- loc_theta: " + str(self.theta)


class Charging_RT:
    def __init__(self):
        self.x = 2.70
        self.y = 0
        self.theta = math.pi
        self.v_theta = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.ros_init()
        self.loc_ = Robot_loc().update_loc_2("map", "back_flag")
        rospy.Subscriber("charging_tf_init", Bool, self.tf_init)

    def ros_init(self):
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/video_tf', Float32MultiArray, self.update)
        t_ = threading.Thread(target=self.tf_publish)
        t_.start()

    def update(self, msg):
        if math.fabs(self.loc_.update_loc_2('charging_flag', 'back_flag').x) > 0.15:
            self.v_x = msg.data[5] / 1000
            self.v_y = -msg.data[3] / 1000
            self.v_theta = -msg.data[1]
            mat44_map_back = tft.euler_matrix(0, 0, self.loc_.update_loc_2("map", "back_flag").theta)
            mat44_map_back[0][3] = self.loc_.update_loc_2("map", "back_flag").x
            mat44_map_back[1][3] = self.loc_.update_loc_2("map", "back_flag").y
            mat44_back_charging = tft.euler_matrix(0, 0, self.v_theta)
            mat44_back_charging[0][3] = self.v_x
            mat44_back_charging[1][3] = self.v_y
            mat44_map_charging = np.dot(mat44_map_back, mat44_back_charging)
            self.x = mat44_map_charging[0][3]
            self.y = mat44_map_charging[1][3]
            self.theta = tft.euler_from_matrix(mat44_map_charging)[2] + math.pi

    def tf_init(self, msg):
        if msg.data:
            self.x = 2.70
            self.y = 0
            self.theta = math.pi

    def tf_publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "charging_flag"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = tft.quaternion_about_axis(self.theta,(0, 0, 1))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.br.sendTransform(t)
            # print t
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('pub_charging_RT', anonymous=False)
        Charging_RT()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
