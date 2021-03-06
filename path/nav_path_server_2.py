#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

""" nav_test.py - Version 1.1 2013-12-20

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

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
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rock_msgs.srv import NavPath, NavPathRequest, NavPathResponse

from std_msgs.msg import Float32MultiArray

from dynamic_reconfigure.server import Server
from random import sample
import tf.transformations as tft

def normalize(z):
    return math.atan2(math.sin(z), math.cos(z))


def angle_diff(a, b):
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

# example, CLR p.528
class Dijkstra_Map:
    def __init__(self):
        self.G = {'z': {'a': 1},
                  'a': {'c': 1, 'f': 1, 'z': 1},
                  'c': {'a': 1, 'e': 1, 'h': 1},
                  'e': {'c': 1},
                  'f': {'a': 1, 'h': 1, 'k': 1},
                  'h': {'c': 1, 'j': 1, 'f': 1, 'm': 1},
                  'j': {'h': 1},
                  'k': {'p': 1, 'f': 1, 'm': 1},
                  'm': {'k': 1, 'o': 1, 'h': 1, 'r': 1},
                  'o': {'m': 1},
                  'p': {'r': 1, 'k': 1},
                  'r': {'t': 1, 'm': 1, 'p': 1},
                  't': {'r': 1}
                  }

        self.G_b = {'z': {'a': 1},
                  'a': {'c': 1, 'f': 1, 'z': 1},
                  'c': {'a': 1, 'e': 1, 'h': 1},
                  'e': {'c': 1},
                  'f': {'a': 1, 'h': 0, 'k': 1},
                  'h': {'c': 1, 'j': 0, 'f': 0, 'm': 1},
                  'j': {'h': 0},
                  'k': {'p': 1, 'f': 1, 'm': 0},
                  'm': {'k': 0, 'o': 0, 'h': 1, 'r': 1},
                  'o': {'m': 0},
                  'p': {'r': 1, 'k': 1},
                  'r': {'t': 1, 'm': 1, 'p': 1},
                  't': {'r': 1}
                  }

        self.G_p = {'a': (0.61, 2.20, 0),
                    'c': (0.67, 5.80, 0),
                    'e': (0.63, 8.76, 0),
                    'f': (2.22, 2.31, 0),
                    'h': (2.27, 5.71, 0),
                    'j': (2.24, 8.73, 0),
                    'k': (3.78, 2.34, 0),
                    'm': (3.70, 5.70, 0),
                    'o': (3.76, 8.76, 0),
                    'p': (5.39, 2.31, 0),
                    'r': (5.38, 5.74, 0),
                    't': (5.38, 8.70, 0),
                    'z': (0.00, 1.50, 0)
                    }

        self.G_rot = {'a': (0.61, 2.20, 0),
                    'c': (0.67, 5.80, 0),
                    'e': (0.63, 8.76, 0),
                    'f': (2.22, 2.31, 0),
                    'h': (2.27, 5.71, 0),
                    'j': (2.24, 8.73, 0),
                    'k': (3.78, 2.34, 0),
                    'm': (3.70, 5.70, 0),
                    'o': (3.76, 8.76, 0),
                    'p': (5.39, 2.31, 0),
                    'r': (5.38, 5.74, 0),
                    't': (5.38, 8.70, 0),
                    }

        for v in self.G_p:
            for w in self.G[v]:
                self.G[v][w] = math.sqrt((self.G_p[v][0] - self.G_p[w][0]) ** 2 + (self.G_p[v][1] - self.G_p[w][1]) ** 2)
                # print G[v][w]

        self.G_Map = copy.deepcopy(self.G)
        self.G_p_Map = copy.deepcopy(self.G_p)

    def add_pos(self, pos, str_pos):
        c_dist = 999
        c_v = 'a'
        c_w = 'z'
        for v in self.G_p:
            for w in self.G[v]:
                dist = point2segdist(pos[0], pos[1], self.G_p[v][0], self.G_p[v][1], self.G_p[w][0], self.G_p[w][1])
                if dist <= c_dist:
                    c_dist = dist
                    c_v = v
                    c_w = w
                    # print G[v][w]
        self.G_Map[c_v][str_pos] = math.sqrt((self.G_p[c_v][0] - pos[0]) ** 2 + (self.G_p[c_v][1] - pos[1]) ** 2)
        self.G_Map[c_w][str_pos] = math.sqrt((self.G_p[c_w][0] - pos[0]) ** 2 + (self.G_p[c_w][1] - pos[1]) ** 2)
        self.G_Map[str_pos] = {}
        self.G_Map[str_pos][c_v] = math.sqrt((self.G_p[c_v][0] - pos[0]) ** 2 + (self.G_p[c_v][1] - pos[1]) ** 2)
        self.G_Map[str_pos][c_w] = math.sqrt((self.G_p[c_w][0] - pos[0]) ** 2 + (self.G_p[c_w][1] - pos[1]) ** 2)
        self.G_p_Map[str_pos] = pos

    def add_start_pos(self, pos):
        # pos[0]: x pos[1]: y pos[2]: theta
        str_pos = 'start'
        c_dist = 999
        c_v = 'a'
        c_w = 'z'
        for v in self.G_p:
            for w in self.G[v]:
                dist = point2segdist(pos[0], pos[1], self.G_p[v][0], self.G_p[v][1], self.G_p[w][0], self.G_p[w][1])
                if dist <= c_dist:
                    c_dist = dist
                    c_v = v
                    c_w = w
                    # print G[v][w]
        if not self.G_b[c_v][c_w]:
            theta_cv = math.atan2(self.G_p[c_v][1] - pos[1], self.G_p[c_v][0] - pos[0])
            theta_cw = math.atan2(self.G_p[c_w][1] - pos[1], self.G_p[c_w][0] - pos[0])
            if angle_diff(theta_cv, pos[2]) > angle_diff(theta_cw, pos[2]):
                self.G_Map[c_v][c_w] = 999
                self.G_Map[c_w][c_v] = 999
                self.G_Map[c_w][str_pos] = math.sqrt(
                    (self.G_p[c_w][0] - pos[0]) ** 2 + (self.G_p[c_w][1] - pos[1]) ** 2)
                self.G_Map[str_pos] = {}
                self.G_Map[str_pos][c_w] = math.sqrt(
                    (self.G_p[c_w][0] - pos[0]) ** 2 + (self.G_p[c_w][1] - pos[1]) ** 2)
                self.G_p_Map[str_pos] = pos
            else:
                self.G_Map[c_w][c_v] = 999
                self.G_Map[c_v][c_w] = 999
                self.G_Map[c_v][str_pos] = math.sqrt(
                    (self.G_p[c_v][0] - pos[0]) ** 2 + (self.G_p[c_v][1] - pos[1]) ** 2)
                self.G_Map[str_pos] = {}
                self.G_Map[str_pos][c_v] = math.sqrt(
                    (self.G_p[c_v][0] - pos[0]) ** 2 + (self.G_p[c_v][1] - pos[1]) ** 2)
                self.G_p_Map[str_pos] = pos
        else:
            self.G_Map[c_v][str_pos] = math.sqrt((self.G_p[c_v][0] - pos[0]) ** 2 + (self.G_p[c_v][1] - pos[1]) ** 2)
            self.G_Map[c_w][str_pos] = math.sqrt((self.G_p[c_w][0] - pos[0]) ** 2 + (self.G_p[c_w][1] - pos[1]) ** 2)
            self.G_Map[str_pos] = {}
            self.G_Map[str_pos][c_v] = math.sqrt((self.G_p[c_v][0] - pos[0]) ** 2 + (self.G_p[c_v][1] - pos[1]) ** 2)
            self.G_Map[str_pos][c_w] = math.sqrt((self.G_p[c_w][0] - pos[0]) ** 2 + (self.G_p[c_w][1] - pos[1]) ** 2)
            self.G_p_Map[str_pos] = pos

    def get_path(self, start, stop):
        self.add_start_pos(start)
        self.add_pos(stop, 'stop')
        rospy.loginfo("start point is start")
        path = shortestPath(self.G_Map, "start", "stop")
        return path

    def trace_path(self, start, trace_path):
        self.add_pos(start, 'start')
        c_dist = 999
        cloest_pos_str = ''
        for pos_str in trace_path:
            pos = self.G_p[pos_str]
            dist = (start[0] - pos[0]) ** 2 + (start[1] - pos[1]) ** 2
            if dist <= c_dist:
                c_dist = dist
                cloest_pos_str = pos_str
        trace_index = trace_path.index(cloest_pos_str)
        path = trace_path[trace_index: -1] + trace_path[0: trace_index]
        path = path + path
        path.insert(0, 'start')
        path.append(path[1])
        return path

    def get_pose(self, path):
        locations = collections.OrderedDict()
        for path_id in range(len(path)):
            if path_id < len(path) - 1:
                theta = math.atan2(self.G_p_Map[path[path_id + 1]][1] - self.G_p_Map[path[path_id]][1],
                               self.G_p_Map[path[path_id + 1]][0] - self.G_p_Map[path[path_id]][0])
            else:
                theta = math.atan2(self.G_p_Map[path[path_id]][1] - self.G_p_Map[path[path_id - 1]][1],
                               self.G_p_Map[path[path_id]][0] - self.G_p_Map[path[path_id - 1]][0])
            quat_ = tft.quaternion_from_euler(0, 0, theta)
            locations[path[path_id]] = Pose(Point(self.G_p_Map[path[path_id]][0], self.G_p_Map[path[path_id]][1],
                              self.G_p_Map[path[path_id]][2]),
                        Quaternion(quat_[0], quat_[1], quat_[2], quat_[3]))
        return locations

    def get_list_pose(self, path):
        path_list = []
        for path_id in range(len(path)):
            if path_id < len(path) - 1:
                theta = math.atan2(self.G_p_Map[path[path_id + 1]][1] - self.G_p_Map[path[path_id]][1],
                               self.G_p_Map[path[path_id + 1]][0] - self.G_p_Map[path[path_id]][0])
            else:
                theta = math.atan2(self.G_p_Map[path[path_id]][1] - self.G_p_Map[path[path_id - 1]][1],
                               self.G_p_Map[path[path_id]][0] - self.G_p_Map[path[path_id - 1]][0])
            quat_ = tft.quaternion_from_euler(0, 0, theta)
            path_list.append(Pose(Point(self.G_p_Map[path[path_id]][0], self.G_p_Map[path[path_id]][1],
                              self.G_p_Map[path[path_id]][2]),
                        Quaternion(quat_[0], quat_[1], quat_[2], quat_[3])))
        return path_list

    def init_Map(self):
        self.G_Map = copy.deepcopy(self.G)
        self.G_p_Map = copy.deepcopy(self.G_p)


def point2segdist(x, y, x1, y1, x2, y2):
    cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1)
    if cross <= 0:
        dist = math.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1))
        return dist
    d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)
    if cross >= d2:
        dist = math.sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))
        return dist
    r = cross / d2
    px = x1 + (x2 - x1) * r
    py = y1 + (y2 - y1) * r
    dist = math.sqrt((x - px) * (x - px) + (py - y) * (py - y))
    return dist


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
                # self.output()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()

    def output(self):
        print "loc_x: " + str(self.x) + "---- loc_y: " + str(self.y) + "---- loc_theta: " + str(self.theta)


class Simple:
    def __init__(self):
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.loc_ = Robot_loc().update_loc("map")

    def go(self, theta, length):
        self.rotate(theta)
        self.translation(length)
        print "!!!!!!!!!!Goal Reached!!!!!!!!!!"

    def rotate(self, theta):  # 右手系，逆时针为正
        old_theta = self.loc_.update_loc("map").theta
        rate = rospy.Rate(10)
        if theta > 0:
            while angle_diff(self.loc_.update_loc("map").theta, old_theta) < theta:
                self.cmd_vel(0, 0.2)
                rate.sleep()
        elif theta < 0:
            while angle_diff(self.loc_.update_loc("map").theta, old_theta) > theta:
                self.cmd_vel(0, -0.2)
                rate.sleep()
        self.cmd_vel(0, 0)

    def rotate_trace(self, theta, trace_frame):  # 右手系，逆时针为正
        # TODO 可能会因为角度-pi~pi的问题引发新的问题。
        d_theta = 0.1
        old_theta = self.loc_.update_loc(trace_frame).theta
        rate = rospy.Rate(10)
        if old_theta < theta:
            while self.loc_.update_loc(trace_frame).theta < theta:
                k = (abs(self.loc_.update_loc(trace_frame).theta))
                if k < d_theta:
                    break
                if k <= 0.17:
                    k = 0.17
                self.cmd_vel(0, k)
                rate.sleep()
        elif old_theta > theta:
            while self.loc_.update_loc(trace_frame).theta > theta - d_theta:
                k = (abs(self.loc_.update_loc(trace_frame).theta))
                if k < d_theta:
                    break
                if k <= 0.17:
                    k = 0.17
                self.cmd_vel(0, -k)
                rate.sleep()
        self.cmd_vel(0, 0)

    def translation(self, length):  # 右手系，逆时针为正
        rate = rospy.Rate(10)
        old_x = self.loc_.update_loc("map").x
        old_y = self.loc_.update_loc("map").y
        while (self.loc_.update_loc("map").x - old_x) ** 2 + \
                        (self.loc_.update_loc("map").y - old_y) ** 2 < length ** 2:
            self.cmd_vel(0.2, 0)
            # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
            rate.sleep()
        self.cmd_vel(0, 0)

    # def translation_trace(self, length): # 右手系，逆时针为正
    #     rate = rospy.Rate(10)
    #     old_x = self.loc_.update_loc("map").x
    #     old_y = self.loc_.update_loc("map").y
    #     while (self.loc_.update_loc("map").x - old_x) ** 2 + (self.loc_.update_loc("map").y - old_y) ** 2 < length ** 2:
    #         self.cmd_vel(0.2, 0)
    #         # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
    #         rate.sleep()
    #     self.cmd_vel(0, 0)

    def cmd_vel(self, linear, angular):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear
        cmd_msg.angular.z = angular
        self.pub_vel.publish(cmd_msg)


class NavProcess():
    def __init__(self):

        rospy.on_shutdown(self.shutdown)

        self.loc = Robot_loc()
        self.simple = Simple()
        self.path_map = Dijkstra_Map()

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 2)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", True)

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.

    def go_stop(self, stop):
        path = self.path_map.get_path((self.loc.update_loc('map').x, self.loc.update_loc('map').y, self.loc.update_loc('map').theta), stop)
        rospy.loginfo("path is " + str(path))
        if len(path) > 1:
            locations = self.path_map.get_pose(path)

            self.path_map.init_Map()
            # print locations

            # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
            self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

            # Subscribe to the move_base action server
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

            rospy.loginfo("Waiting for move_base action server...")

            # Wait 60 seconds for the action server to become available
            self.move_base.wait_for_server(rospy.Duration(60))

            rospy.loginfo("Connected to move base server")

            # A variable to hold the initial pose of the robot to be set by
            # the user in RViz
            # initial_pose = PoseWithCovarianceStamped()

            # Get the initial pose from the user
            # rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
            # rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
            # self.last_location = Pose()
            # rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

            # Make sure we have the initial pose
            # while initial_pose.header.stamp == "":
            #     rospy.sleep(1)

            rospy.loginfo("Starting navigation test")

            for location in locations.keys():
                # Set up the next goal location
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = locations[location]
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()
                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + str(location))

                # Start the robot toward the next location
                self.move_base.send_goal(self.goal)

                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        rospy.loginfo("State:" + str(state))
                    else:
                        rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

                # Print a summary success/failure, distance traveled and time elapsed
                rospy.sleep(self.rest_time)
            rospy.loginfo("robot is on this point")
        else:
            rospy.loginfo("robot is on this point")

    def trace_path(self, trace_path):
        path = self.path_map.trace_path(
            (self.loc.update_loc('map').x, self.loc.update_loc('map').y, self.loc.update_loc('map').theta), trace_path)
        rospy.loginfo("path is " + str(path))
        if len(path) > 1:
            path_list = self.path_map.get_list_pose(path)

            self.path_map.init_Map()
            # print locations

            # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
            self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

            # Subscribe to the move_base action server
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

            rospy.loginfo("Waiting for move_base action server...")

            # Wait 60 seconds for the action server to become available
            self.move_base.wait_for_server(rospy.Duration(60))

            rospy.loginfo("Connected to move base server")

            # A variable to hold the initial pose of the robot to be set by
            # the user in RViz
            # initial_pose = PoseWithCovarianceStamped()

            # Get the initial pose from the user
            # rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
            # rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
            # self.last_location = Pose()
            # rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

            # Make sure we have the initial pose
            # while initial_pose.header.stamp == "":
            #     rospy.sleep(1)

            rospy.loginfo("Starting navigation test")

            for location in path_list:
                # Set up the next goal location
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = location
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()
                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + path[path_list.index(location)])

                # Start the robot toward the next location
                self.move_base.send_goal(self.goal)

                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        rospy.loginfo("State:" + str(state))
                    else:
                        rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

                # Print a summary success/failure, distance traveled and time elapsed
                rospy.sleep(self.rest_time)
            rospy.loginfo("robot is on this point")
        else:
            rospy.loginfo("robot is on this point")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


def handle_nav_path(req):
    rospy.loginfo("recieve requset")
    nav_process.go_stop((req.navgoal.x, req.navgoal.y, req.navgoal.z))
    return NavPathResponse(True)


def nav_path_server():
    s = rospy.Service('nav_path', NavPath, handle_nav_path)
    rospy.loginfo("server ready.")


if __name__ == '__main__':
    try:
        rospy.init_node('nav_path_server', anonymous=True)
        nav_process = NavProcess()
        nav_path_server()
        # NavProcess((4, 6, 0))
        # path = ['a', 'f', 'k', 'm', 'h', 'c', 'a']
        # nav_process.trace_path(path)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
