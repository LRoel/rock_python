#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
import numpy.linalg as la
import math
import time
import tf2_ros

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server

import tf.transformations as tft


def normalize(z):
    """归一化

    Args:
        z: 角度输入

    Returns:
        角度输出(0~pi)

    """

    return math.atan2(math.sin(z), math.cos(z))


def angle_diff(a, b):
    """角度差值

    Args:
        a: 输入角度
        b: 输入角度

    Returns:
        角度差值

    """

    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2


def matrix_from_theta(theta):
    """角度生成旋转矩阵(2x2)

    Args:
        theta: 输入角度

    Returns:
        输出旋转矩阵

    """

    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def theta_from_matrix(R):
    """旋转矩阵(2x2)转成角度

    Args:
        R: 输入旋转矩阵

    Returns:
        输出角度值

    """

    _R_12 = R[0, 1]
    _R_22 = R[1, 1]
    if -0.001 < _R_22 < 0.001 and _R_12 > 0:
        theta = math.pi / 2
    elif -0.001 < _R_22 < 0.001 and _R_12 < 0:
        theta = math.pi / 2
    elif 0.001 <= _R_22 and _R_12 > 0:
        theta = math.atan(_R_12 / _R_22)
    elif 0.001 <= _R_22 and _R_12 < 0:
        theta = math.atan(_R_12 / _R_22)
    elif _R_22 <= -0.001 and _R_12 > 0:
        theta = math.atan(_R_12 / _R_22) + math.pi
    elif _R_22 <= -0.001 and _R_12 < 0:
        theta = math.atan(_R_12 / _R_22) - math.pi
    return theta


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

    def output(self):
        print "loc_x: " + str(self.x) + "---- loc_y: " + str(self.y) + "---- loc_theta: " + str(self.theta)


class Job_move():

    def __init__(self):
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        thread_move = threading.Thread(target=self.run)
        thread_move.start()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_vel.publish(self.cmd_msg)
            rate.sleep()

    def move_stop(self):
        cmd_msg_zero_ = Twist()
        self.pub_vel.publish(cmd_msg_zero_)

    def cmd_vel(self, linear, angular):
        self.cmd_msg.linear.x = linear
        self.cmd_msg.angular.z = angular


class Simple:
    def __init__(self):
        rospy.init_node('simple_move')
        self.move = Job_move()
        self.move.run()
        self.loc_ = Robot_loc().update_loc("map")

    # def go(self, theta, length):
    #     self.rotate(theta)
    #     self.translation(length)
    #     print "!!!!!!!!!!Goal Reached!!!!!!!!!!"
    #
    # def go_zero_frame(self, trace_frame):
    #     n = 0
    #     if self.loc_.update_loc(trace_frame).y < 0:
    #         k = 3*(abs(self.loc_.update_loc(trace_frame).y))
    #         self.rotate_trace(k * math.pi/4, trace_frame)
    #         rate = rospy.Rate(10)
    #         while self.loc_.update_loc(trace_frame).y < 0 and n < 2:
    #             self.move.cmd_vel(0.5, 0)
    #             n += 1
    #             # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
    #             rate.sleep()
    #         self.move.cmd_vel(0, 0)
    #         # self.rotate_trace(0, trace_frame)
    #     elif self.loc_.update_loc(trace_frame).y > 0:
    #         k = (abs(self.loc_.update_loc(trace_frame).y))
    #         self.rotate_trace(k * -math.pi/4, trace_frame)
    #         rate = rospy.Rate(10)
    #         while self.loc_.update_loc(trace_frame).y > 0 and n < 2:
    #             self.move.cmd_vel(0.5, 0)
    #             n += 1
    #             # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
    #             rate.sleep()
    #         self.move.cmd_vel(0, 0)
    #         # self.rotate_trace(0, trace_frame)
    #
    # def go_zero_frame_2(self, trace_frame):
    #
    #     if self.loc_.update_loc(trace_frame).y < 0:
    #         k = 10*(abs(self.loc_.update_loc(trace_frame).y))**3
    #         rate = rospy.Rate(10)
    #         n = 0
    #         while self.loc_.update_loc(trace_frame).y < 0:
    #             self.move.cmd_vel(0.5, k)
    #             n += 1
    #             if n >= 2:
    #                 break
    #             # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
    #             rate.sleep()
    #         self.move.cmd_vel(0, 0)
    #         if n < 2:
    #             self.rotate_trace(0, trace_frame)
    #     elif self.loc_.update_loc(trace_frame).y > 0:
    #         k = 10*(abs(self.loc_.update_loc(trace_frame).y))**3
    #         rate = rospy.Rate(10)
    #         n = 0
    #         while self.loc_.update_loc(trace_frame).y > 0:
    #             self.move.cmd_vel(0.5, -k)
    #             n += 1
    #             if n >= 2:
    #                 break
    #             # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
    #             rate.sleep()
    #         self.move.cmd_vel(0, 0)
    #         if n < 2:
    #             self.rotate_trace(0, trace_frame)

    def rotate(self, theta): # 右手系，逆时针为正
        old_theta = self.loc_.update_loc("map").theta
        self.move.resume()
        rate = rospy.Rate(10)
        if theta > 0:
            while angle_diff(self.loc_.update_loc("map").theta, old_theta) < theta:
                self.move.cmd_msg.angular.z = 0.5
                rate.sleep()
        elif theta < 0:
            while angle_diff(self.loc_.update_loc("map").theta ,old_theta) > theta:
                self.move.cmd_msg.angular.z = -0.5
                rate.sleep()
        self.move.cmd_msg.angular.z = 0.0
        self.move.pause()

    # def rotate_trace(self, theta, trace_frame): # 右手系，逆时针为正
    #     # TODO 可能会因为角度-pi~pi的问题引发新的问题。
    #     old_theta = self.loc_.update_loc(trace_frame).theta
    #     rate = rospy.Rate(10)
    #     if theta > old_theta:
    #         while self.loc_.update_loc(trace_frame).theta < theta:
    #             self.move.cmd_vel(0, 0.5)
    #             rate.sleep()
    #     elif theta < old_theta:
    #         while self.loc_.update_loc(trace_frame).theta > theta:
    #             self.move.cmd_vel(0, -0.5)
    #             rate.sleep()
    #     self.move.cmd_vel(0, 0)

    def translation(self, length): # 右手系，逆时针为正
        rate = rospy.Rate(10)
        old_x = self.loc_.update_loc("map").x
        old_y = self.loc_.update_loc("map").y
        while (self.loc_.update_loc("map").x - old_x) ** 2 + (self.loc_.update_loc("map").y - old_y) ** 2 < length ** 2:
            self.move.cmd_msg.linear.x = 0.5
            # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
            rate.sleep()
        self.move.cmd_msg.linear.x = 0.0

    # def translation_trace(self, length): # 右手系，逆时针为正
    #     rate = rospy.Rate(10)
    #     old_x = self.loc_.update_loc("map").x
    #     old_y = self.loc_.update_loc("map").y
    #     while (self.loc_.update_loc("map").x - old_x) ** 2 + (self.loc_.update_loc("map").y - old_y) ** 2 < length ** 2:
    #         self.cmd_vel(0.5, 0)
    #         # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
    #         rate.sleep()
    #     self.cmd_vel(0, 0)


if __name__ == '__main__':
    tmp = Simple()
    while not tmp.loc_.update_loc("map").ok():
        print "waiting"

    while not rospy.is_shutdown():
        tmp.translation(1)