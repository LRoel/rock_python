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

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

from rock_msgs.srv import *

from dynamic_reconfigure.server import Server
from random import sample
import tf.transformations as tft
from std_msgs.msg import Bool


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


class State_stm32:
    def __init__(self):
        self.charging_flag_ok = False
        self.docking_flag_ok = False
        rospy.Subscriber('stm32_data', Int32MultiArray, self.update)

    def update(self, msg):
        # msg = Float32MultiArray()
        # TODO msg.data[12]展示为充电方向，不是充电成功标志，要做个选择，具体和鑫磊商讨。
        self.charging_flag_ok = msg.data[7]
        # TODO msg.data[13]为connection状态，状态变化情况和鑫磊商讨。
        # 3:未连接 2：连接
        if msg.data[8] == 3:
            self.docking_flag_ok = False
        elif msg.data[8] == 2:
            self.docking_flag_ok = True
        # self.docking_flag_ok = msg.data[13]
        # print self.docking_flag_ok
        # print self.charging_flag_ok


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


# class Charging_RT:
#     def __init__(self):
#         self.theta = 0.0
#         self.x = 0.0
#         self.y = 0.0
#         self.v_theta = 0.0
#         self.v_x = 0.0
#         self.v_y = 0.0
#         self.ros_init()
#         self.loc_ = Robot_loc().update_loc_2("map", "back_flag")
#
#     def ros_init(self):
#         self.br = tf2_ros.TransformBroadcaster()
#         rospy.Subscriber('/video_tf', Float32MultiArray, self.update)
#         t_ = threading.Thread(target=self.tf_publish)
#         t_.start()
#
#     def update(self, msg):
#         self.v_x = msg.data[5]/1000
#         self.v_y = msg.data[3]/1000
#         self.v_theta = msg.data[1]
#         mat44_map_back = tft.euler_matrix(0, 0, self.loc_.update_loc_2("map", "back_flag").theta)
#         mat44_map_back[0][3] = self.loc_.update_loc_2("map", "back_flag").x
#         mat44_map_back[1][3] = self.loc_.update_loc_2("map", "back_flag").y
#         mat44_back_charging = tft.euler_matrix(0, 0, self.v_theta)
#         mat44_back_charging[0][3] = self.v_x
#         mat44_back_charging[1][3] = self.v_y
#         mat44_map_charging = np.dot(mat44_map_back, mat44_back_charging)
#         self.x = mat44_map_charging[0][3]
#         self.y = mat44_map_charging[1][3]
#         self.theta = tft.euler_from_matrix(mat44_map_charging)[2] + math.pi
#
#     def tf_publish(self):
#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             t = TransformStamped()
#             t.header.stamp = rospy.Time.now()
#             t.header.frame_id = "map"
#             t.child_frame_id = "charging_flag"
#             t.transform.translation.x = self.x
#             t.transform.translation.y = self.y
#             t.transform.translation.z = 0.0
#             q = tft.quaternion_about_axis(self.theta,(0, 0, 1))
#             t.transform.rotation.x = q[0]
#             t.transform.rotation.y = q[1]
#             t.transform.rotation.z = q[2]
#             t.transform.rotation.w = q[3]
#             self.br.sendTransform(t)
#             print t
#             rate.sleep()


class Simple:
    def __init__(self):
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.loc_ = Robot_loc().update_loc("map")
        self.shutdown_flag = False

    def go(self, theta, length):
        self.rotate(theta)
        self.translation(length)
        rospy.loginfo("!!!!!!!!!!Goal Reached!!!!!!!!!!")

    def go_zero_frame(self, trace_frame):
        rate = rospy.Rate(10)
        if math.fabs(self.loc_.update_loc(
                trace_frame).x) > 0.3 and not robot_stm32.docking_flag_ok and not self.shutdown_flag:
            # print self.loc_.update_loc_2(trace_frame, 'back_flag').y
            if self.loc_.update_loc_2(trace_frame, 'base_footprint').y < -0.02:
                rospy.loginfo('in 1 if')
                l_flag = 0
                while (self.loc_.update_loc_2(trace_frame, 'base_footprint').y < -0.02) \
                        and math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3 \
                        and not robot_stm32.docking_flag_ok and not self.shutdown_flag:
                    rospy.loginfo('in while')
                    if l_flag == 1 or math.fabs(
                                    math.fabs(self.loc_.update_loc(trace_frame).theta) - math.pi / 4) <= 0.1:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'base_footprint').x) > 0.15:
                            self.cmd_vel(-0.08, 0.0)
                        else:
                            self.cmd_vel(-0.08, 0.0)
                        l_flag = 1
                    else:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'base_footprint').x) > 0.15:
                            self.cmd_vel(-0.08, -0.05 - math.fabs(self.loc_.update_loc(trace_frame).y) / 2)
                        else:
                            self.cmd_vel(-0.08, -0.00)
                    # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
                rospy.loginfo('in 1 cmd0')
                self.cmd_vel(0, 0)
                self.rotate_trace(0, trace_frame)
                time.sleep(4)
            elif self.loc_.update_loc_2(trace_frame, 'base_footprint').y > 0.02:
                r_flag = 0
                rospy.loginfo(self.loc_.update_loc_2(trace_frame, 'base_footprint').y > 0.02)
                rospy.loginfo(math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3)
                rospy.loginfo(not robot_stm32.docking_flag_ok)
                while (self.loc_.update_loc_2(trace_frame, 'base_footprint').y > 0.02) \
                        and math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3 \
                        and not robot_stm32.docking_flag_ok and not self.shutdown_flag:

                    if r_flag == 1 or math.fabs(
                                    math.fabs(self.loc_.update_loc(trace_frame).theta) - math.pi / 4) <= 0.1:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'base_footprint').x) > 0.15:
                            self.cmd_vel(-0.08, 0.0)
                        else:
                            self.cmd_vel(-0.08, 0.0)
                        r_flag = 1
                    else:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'base_footprint').x) > 0.15:
                            self.cmd_vel(-0.08, 0.05 + math.fabs(self.loc_.update_loc(trace_frame).y) / 2)
                        else:
                            self.cmd_vel(-0.08, 0.00)
                    # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
                rospy.loginfo('in 2 cmd0')
                self.cmd_vel(0, 0)
                self.rotate_trace(0, trace_frame)
                time.sleep(4)
        while math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3 and not robot_stm32.docking_flag_ok and not self.shutdown_flag:
            # print self.loc_.update_loc_2(trace_frame, 'back_flag').y
            if self.loc_.update_loc_2(trace_frame, 'back_flag').y < -0.02:
                rospy.loginfo('in 1 if')
                l_flag = 0
                while (self.loc_.update_loc_2(trace_frame, 'back_flag').y < -0.02) \
                        and math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3 \
                        and not robot_stm32.docking_flag_ok and not self.shutdown_flag:
                    rospy.loginfo('in while')
                    if l_flag == 1 or math.fabs(math.fabs(self.loc_.update_loc(trace_frame).theta) - math.pi/4) <= 0.1:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'back_flag').x) > 0.15:
                            self.cmd_vel(-0.08, 0.0)
                        else:
                            self.cmd_vel(-0.08, 0.0)
                        l_flag = 1
                    else:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'back_flag').x) > 0.15:
                            self.cmd_vel(-0.08, -0.05 - math.fabs(self.loc_.update_loc(trace_frame).y)/2)
                        else:
                            self.cmd_vel(-0.08, -0.00)
                    # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
                    rate.sleep()
                rospy.loginfo('in 1 cmd0')
                self.cmd_vel(0, 0)
                self.rotate_trace(0, trace_frame)
                time.sleep(4)
            elif self.loc_.update_loc_2(trace_frame, 'back_flag').y > 0.02:
                r_flag = 0
                rospy.loginfo(self.loc_.update_loc_2(trace_frame, 'back_flag').y > 0.02)
                rospy.loginfo(math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3)
                rospy.loginfo(not robot_stm32.docking_flag_ok)
                while (self.loc_.update_loc_2(trace_frame, 'back_flag').y > 0.02) \
                        and math.fabs(self.loc_.update_loc(trace_frame).x) > 0.3 \
                        and not robot_stm32.docking_flag_ok and not self.shutdown_flag:
                    
                    if r_flag == 1 or math.fabs(math.fabs(self.loc_.update_loc(trace_frame).theta) - math.pi/4) <= 0.1:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'back_flag').x) > 0.15:
                            self.cmd_vel(-0.08, 0.0)
                        else:
                            self.cmd_vel(-0.08, 0.0)
                        r_flag = 1
                    else:
                        if math.fabs(self.loc_.update_loc_2(trace_frame, 'back_flag').x) > 0.15:
                            self.cmd_vel(-0.08, 0.05 + math.fabs(self.loc_.update_loc(trace_frame).y)/2)
                        else:
                            self.cmd_vel(-0.08, 0.00)
                    # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
                    rate.sleep()
                rospy.loginfo('in 2 cmd0')
                self.cmd_vel(0, 0)
                self.rotate_trace(0, trace_frame)
                time.sleep(4)
            else:
                if self.loc_.update_loc_2(trace_frame, 'back_flag').x > 0.15:
                    self.cmd_vel(-0.08, 0)
                else:
                    self.cmd_vel(-0.08, 0)
                rate.sleep()
            # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
        rospy.loginfo('in 3 cmd0')
        self.cmd_vel(0, 0)

    def rotate(self, theta): # 右手系，逆时针为正
        old_theta = self.loc_.update_loc("map").theta
        rate = rospy.Rate(10)
        if theta > 0:
            while (angle_diff(self.loc_.update_loc("map").theta, old_theta)  < theta) and not self.shutdown_flag:
                self.cmd_vel(0, 0.4)
                rate.sleep()
        elif theta < 0:
            while (angle_diff(self.loc_.update_loc("map").theta ,old_theta) > theta) and not self.shutdown_flag:
                self.cmd_vel(0, -0.4)
                rate.sleep()
        self.cmd_vel(0, 0)

    def rotate_trace(self, theta, trace_frame): # 右手系，逆时针为正
        # TODO 可能会因为角度-pi~pi的问题引发新的问题。
        d_theta = 0.1
        old_theta = self.loc_.update_loc(trace_frame).theta
        rate = rospy.Rate(10)
        if old_theta < theta:
            while (self.loc_.update_loc(trace_frame).theta < theta + d_theta) and not self.shutdown_flag:
                k = (abs(self.loc_.update_loc(trace_frame).theta))
                if k < d_theta:
                    break
                if k <= 0.17:
                    k = 0.17
                self.cmd_vel(0, k)
                rate.sleep()
        elif old_theta > theta:
            while (self.loc_.update_loc(trace_frame).theta > theta - d_theta) and not self.shutdown_flag:
                k = (abs(self.loc_.update_loc(trace_frame).theta))
                if k < d_theta:
                    break
                if k <= 0.17:
                    k = 0.17
                self.cmd_vel(0, -k)
                rate.sleep()
        self.cmd_vel(0, 0)

    def translation(self, length): # 右手系，逆时针为正
        rate = rospy.Rate(10)
        old_x = self.loc_.update_loc("map").x
        old_y = self.loc_.update_loc("map").y
        while ((self.loc_.update_loc("map").x - old_x) ** 2 +
                        (self.loc_.update_loc("map").y - old_y) ** 2 < length ** 2) and not self.shutdown_flag:
            if length >= 0:
                self.cmd_vel(0.2, 0)
            else:
                self.cmd_vel(-0.2, 0)
            # print (self.loc_.x - old_x) ** 2 + (self.loc_.y - old_y) ** 2
            rate.sleep()
        self.cmd_vel(0, 0)

    def cmd_vel(self, linear, angular):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear
        cmd_msg.angular.z = angular
        self.pub_vel.publish(cmd_msg)

    def shutdown(self):
        self.cmd_vel(0, 0)
        self.shutdown_flag = True
        time.sleep(2)
        self.shutdown_flag = False


class MoveBasePile:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

    def set_goal(self, x, y, theta):
        q_ = tft.quaternion_from_euler(0, 0, theta)
        waypoint = Pose(Point(x, y, 0.0), Quaternion(q_[0], q_[1], q_[2], q_[3]))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'charging_flag'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                rospy.loginfo("State:" + str(state))
                return True
            else:
                rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
                return False

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(5)
        print 'ooookkKK'


docking_pid = 0
def docking():
    docking_pid_cur = docking_pid
    while not robot_stm32.charging_flag_ok and docking_pid_cur == docking_pid:
        robot_sp.shutdown_flag = False
        init_tf_msg = Bool()
        init_tf_msg.data = True
        charging_init_pub.publish(init_tf_msg)
        if docking_pid_cur != docking_pid:
            break
        time.sleep(4)
        move_pile.set_goal(2, 0.0, 0.0)
        if docking_pid_cur != docking_pid:
            break
        time.sleep(4)
        move_pile.set_goal(2, 0.0, 0.0)
        pub = rospy.Publisher('charging', String, queue_size=10)
        tmp = String()
        tmp.data = '\x00'
        pub.publish(tmp)
        time.sleep(2)
        pub.publish(tmp)
        rospy.loginfo('lighting ok')
        time.sleep(2)
        rospy.loginfo('start going')
        robot_sp.go_zero_frame("charging_flag")
        time.sleep(30)
        rospy.loginfo("Not charging ok, retry")
    rospy.loginfo("charging ok")


def docking_stop():
    global docking_stop_flag
    robot_sp.shutdown()
    move_pile.shutdown()
    docking_stop_flag = True
    rospy.loginfo("stop ok")


def handle_simple_move(req):
    robot_sp.shutdown()
    if req.Mode == 0:
        robot_sp.go(req.Radian, req.Meter)
    elif req.Mode == 1:
        robot_sp.translation(req.Meter)
    elif req.Mode == 2:
        robot_sp.rotate(req.Radian)
    elif req.Mode == 3:
        robot_sp.rotate_trace(req.Radian, req.Coordinate)
    else:
        return SimpleMoveRequest(False)
    return SimpleMoveResponse(True)


def handle_auto_charging(req):
    global docking_pid
    if req.low_power:
        rospy.loginfo('start docking')
        docking_stop()
        docking_pid += 1
        docking()
        return AutoChargingResponse(True)
    else:
        docking_stop()
        return AutoChargingResponse(False)


def simple_move_server():
    s = rospy.Service('simple_move', SimpleMove, handle_simple_move)


def auto_charging_server():
    rospy.loginfo('charging server start')
    s = rospy.Service('auto_charging', AutoCharging, handle_auto_charging)


if __name__ == '__main__':
    try:
        charging_flag_ok = False
        docking_flag_ok = False
        rospy.init_node('nav_test', anonymous=False)
        robot_stm32 = State_stm32()
        global robot_sp, docking_stop_flag
        docking_stop_flag = False
        robot_sp = Simple()
        while not robot_sp.loc_.update_loc("map").ok():
            rospy.loginfo("waiting")
        simple_move_server()
        auto_charging_server()
        move_pile = MoveBasePile()
        charging_init_pub = rospy.Publisher('charging_tf_init', Bool, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
