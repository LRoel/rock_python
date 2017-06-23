#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
from dijkstra import shortestPath
import collections
import threading
import numpy as np
import numpy.linalg as la
import math
import copy
import time
import tf2_ros
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, TransformStamped, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rock_msgs.srv import *

from std_msgs.msg import Int32MultiArray

from dynamic_reconfigure.server import Server
from random import sample
import tf.transformations as tft


def nav_path_client(nav_path_req):
    rospy.wait_for_service('nav_path')
    try:
        nav_path = rospy.ServiceProxy('nav_path', NavPath)
        resp1 = nav_path(nav_path_req)
        return resp1.success
    except rospy.ServiceException, e:
        print "Nav Path Service call failed: %s"%e


def simple_move_client(simple_move_req):
    rospy.wait_for_service('simple_move')
    try:
        simple_move = rospy.ServiceProxy('simple_move', SimpleMove)
        resp1 = simple_move(simple_move_req)
        return resp1.success
    except rospy.ServiceException, e:
        print "Simple Move Service call failed: %s" % e


def auto_charging_client(auto_charging_req):
    rospy.wait_for_service('auto_charging')
    try:
        auto_charging = rospy.ServiceProxy('auto_charging', AutoCharging)
        resp1 = auto_charging(auto_charging_req)
        return resp1.success
    except rospy.ServiceException, e:
        print "Auto Charging Service call failed: %s" % e


class Point_test:
    def __init__(self):
        rospy.Subscriber('clicked_point', PointStamped, self.update)

    def update(self, msg):
        tmp = nav_path_client(msg.point)
        if tmp:
            rospy.loginfo("ok")


class Simple_Move:
    def __init__(self):
        self.simple_msg = SimpleMoveRequest()

    def go(self, theta, length):
        self.simple_msg.Mode = 0
        self.simple_msg.Meter = length
        self.simple_msg.Radian = theta
        self.simple_msg.Coordinate = 'map'
        simple_move_client(self.simple_msg)

    def rotate(self, theta): # 右手系，逆时针为正
        self.simple_msg.Mode = 2
        self.simple_msg.Meter = 0
        self.simple_msg.Radian = theta
        self.simple_msg.Coordinate = 'map'
        simple_move_client(self.simple_msg)

    def rotate_trace(self, theta, trace_frame): # 右手系，逆时针为正
        self.simple_msg.Mode = 3
        self.simple_msg.Meter = 0
        self.simple_msg.Radian = theta
        self.simple_msg.Coordinate = trace_frame
        simple_move_client(self.simple_msg)


    def translation(self, length): # 右手系，逆时针为正
        self.simple_msg.Mode = 1
        self.simple_msg.Meter = length
        self.simple_msg.Radian = 0
        self.simple_msg.Coordinate = 'map'
        simple_move_client(self.simple_msg)


class Auto_Charging:
    def __init__(self):
        self.charging_msg = AutoChargingRequest()
        self.charging_status = 0 # 0:尚未充电 1:充电过程中
        self.charging_flag = 0
        self.volta = 0
        rospy.Subscriber('stm32_data', Int32MultiArray, self.update)
        t1 = threading.Thread(target=self.judge)
        t1.start()

    def update(self, msg):
        self.volta = msg.data[6]
        self.charging_status = msg.data[7]

    def judge(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.charging_status:
                if self.volta < 40 and not self.charging_flag:
                    self.charging_flag = 1
                    self.charging()
                if self.volta > 90:
                    self.charging_flag = 0
            else:
                self.charging_flag = 1
            rate.sleep()

    def charging(self):
        self.charging_msg.low_power = True
        auto_charging_client(self.charging_msg)

    def uncharging(self):
        self.charging_msg.low_power = False
        auto_charging_client(self.charging_msg)


def process():
    charging_ = Auto_Charging()
    while not rospy.is_shutdown():
        if charging_.charging_flag:
            break



if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True)
        tmp = Point_test()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
