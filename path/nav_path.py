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

from std_msgs.msg import Float32MultiArray

from dynamic_reconfigure.server import Server
from random import sample
import tf.transformations as tft

# example, CLR p.528
G = {'z': {'a': 1},
     'a': {'b': 1, 'f': 1, 'z': 1},
     'b': {'a': 1, 'c': 1},
     'c': {'b': 1, 'd': 1, 'h': 1},
     'd': {'c': 1, 'e': 1},
     'e': {'d': 1},
     'f': {'a': 1, 'g': 1, 'k': 1},
     'g': {'f': 1, 'h': 1},
     'h': {'c': 1, 'i': 1, 'g': 1, 'm': 1},
     'i': {'h': 1, 'j': 1},
     'j': {'i': 1},
     'k': {'p': 1, 'f': 1, 'l': 1},
     'l': {'m': 1, 'k': 1},
     'm': {'l': 1, 'n': 1, 'h': 1, 'r': 1},
     'n': {'o': 1, 'm': 1},
     'o': {'n': 1},
     'p': {'q': 1, 'k': 1},
     'q': {'r': 1, 'p': 1},
     'r': {'s': 1, 'm': 1, 'q': 1},
     's': {'t': 1, 'r': 1},
     't': {'s': 1}
     }

"""
0.61	0.61	0.67	0.67	0.63	2.22	2.23	2.27	2.24	2.24	3.78	3.75	3.7	3.73	3.76	5.39	5.36	5.38	5.38	5.38
2.2	    3.89	5.8	    7.35	8.76	2.31	4.02	5.71	7.31	8.73	2.34	4.12	5.7	7.31	8.76	2.31	4.19	5.74	7.31	8.7

"""
G_p = {'a': (0.61, 2.20, 0),
       'b': (0.61, 3.89, 0),
       'c': (0.67, 5.80, 0),
       'd': (0.67, 7.35, 0),
       'e': (0.63, 8.76, 0),
       'f': (2.22, 2.31, 0),
       'g': (2.23, 4.02, 0),
       'h': (2.27, 5.71, 0),
       'i': (2.24, 7.31, 0),
       'j': (2.24, 8.73, 0),
       'k': (3.78, 2.34, 0),
       'l': (3.75, 4.12, 0),
       'm': (3.70, 5.70, 0),
       'n': (3.73, 7.31, 0),
       'o': (3.76, 8.76, 0),
       'p': (5.39, 2.31, 0),
       'q': (5.36, 4.19, 0),
       'r': (5.38, 5.74, 0),
       's': (5.38, 7.31, 0),
       't': (5.38, 8.70, 0),
       'z': (0.00, 1.50, 0)
       }

for v in G_p:
    for w in G[v]:
        G[v][w] = math.sqrt((G_p[v][0] - G_p[w][0]) ** 2 + (G_p[v][1] - G_p[w][1]) ** 2)
        # print G[v][w]

G_Map = copy.deepcopy(G)
G_p_Map = copy.deepcopy(G_p)


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


def add_pos(pos, str_pos):
    c_dist = 999
    c_v = 'a'
    c_w = 'z'
    for v in G_p:
        for w in G[v]:
            dist = point2segdist(pos[0], pos[1], G_p[v][0], G_p[v][1], G_p[w][0], G_p[w][1])
            if dist <= c_dist:
                c_dist = dist
                c_v = v
                c_w = w
                # print G[v][w]
    G_Map[c_v][str_pos] = math.sqrt((G_p[c_v][0] - pos[0]) ** 2 + (G_p[c_v][1] - pos[1]) ** 2)
    G_Map[c_w][str_pos] = math.sqrt((G_p[c_w][0] - pos[0]) ** 2 + (G_p[c_w][1] - pos[1]) ** 2)
    G_Map[str_pos] = {}
    G_Map[str_pos][c_v] = math.sqrt((G_p[c_v][0] - pos[0]) ** 2 + (G_p[c_v][1] - pos[1]) ** 2)
    G_Map[str_pos][c_w] = math.sqrt((G_p[c_w][0] - pos[0]) ** 2 + (G_p[c_w][1] - pos[1]) ** 2)
    G_p_Map[str_pos] = pos


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


class NavPath():
    def __init__(self, stop):

        rospy.on_shutdown(self.shutdown)

        self.loc = Robot_loc()
        self.simple = Simple()

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 2)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", True)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.

        locations = collections.OrderedDict()
        add_pos((self.loc.update_loc('map').x, self.loc.update_loc('map').y, 0.00), 'start')
        add_pos(stop, 'stop')
        rospy.loginfo("start point is start")
        path = shortestPath(G_Map, "start", "stop")
        rospy.loginfo("path is " + str(path))
        if len(path) > 1:
            for path_id in range(len(path)):
                path_point = path[path_id]
                if path_id < len(path) - 1:
                    theta = math.atan2(G_p_Map[path[path_id + 1]][1] - G_p_Map[path[path_id]][1],
                                       G_p_Map[path[path_id + 1]][0] - G_p_Map[path[path_id]][0])
                    quat_ = tft.quaternion_from_euler(0, 0, theta)
                locations[path_point] = Pose(
                    Point(G_p_Map[path_point][0], G_p_Map[path_point][1], G_p_Map[path_point][2]),
                    Quaternion(quat_[0], quat_[1], quat_[2], quat_[3]))

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

            for location in locations.keys()[1:]:
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
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

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


"""
public static double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)
{
double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);

if (cross <= 0) return Math.Sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
if (cross >= d2) return Math.Sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));

double r = cross / d2;
double px = x1 + (x2 - x1) * r;
double py = y1 + (y2 - y1) * r;
return Math.Sqrt((x - px) * (x - px) + (py - y) * (py - y));
}
"""

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True)
        NavPath((3.80, 7.00, 0.00))
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
