#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import string
import re
import math
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Range


class Serial_32:
    def __init__(self):
        rospy.init_node('serial_fuse', anonymous=True)
        self.pub = rospy.Publisher('stm32_data', Int32MultiArray, queue_size=100)
        self.pub_range1 = rospy.Publisher('sensors/sonar1', Range, queue_size=10)
        self.pub_range2 = rospy.Publisher('sensors/sonar2', Range, queue_size=10)
        self.pub_range3 = rospy.Publisher('sensors/sonar3', Range, queue_size=10)
        self.pub_range4 = rospy.Publisher('sensors/sonar4', Range, queue_size=10)
        self.pub_range5 = rospy.Publisher('sensors/sonar5', Range, queue_size=10)
        self.pub_range6 = rospy.Publisher('sensors/sonar6', Range, queue_size=10)
        rospy.Subscriber("charging", String, self.charge_Callback)
        self.ser = serial.Serial(port='/dev/stm32', baudrate=9600, timeout=1)
        self.ser.flushOutput()
        self.ser.flushInput()
        self.range_sensor_data = [0] * 6
        t1 = threading.Thread(target=self.serial_process)
        t2 = threading.Thread(target=self.range_process)
        t1.start()
        t2.start()

    def charge_Callback(self, msg):
        print msg.data
        data = '\x00'
        self.ser.write(data)
        print data

    def range_process(self):
        range_msg = Range()
        range_msg.radiation_type = range_msg.ULTRASOUND
        range_msg.field_of_view = 60 * math.pi / 180
        range_msg.min_range = 0.20
        range_msg.max_range = 2.0
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "sonar1"
            range_msg.range = self.range_sensor_data[0] / 1000.0
            self.pub_range1.publish(range_msg)
            range_msg.header.frame_id = "sonar2"
            range_msg.range = self.range_sensor_data[1] / 1000.0
            self.pub_range2.publish(range_msg)
            range_msg.header.frame_id = "sonar3"
            range_msg.range = self.range_sensor_data[2] / 1000.0
            self.pub_range3.publish(range_msg)
            range_msg.header.frame_id = "sonar4"
            range_msg.range = self.range_sensor_data[3] / 1000.0
            self.pub_range4.publish(range_msg)
            range_msg.header.frame_id = "sonar5"
            range_msg.range = self.range_sensor_data[4] / 1000.0
            self.pub_range5.publish(range_msg)
            range_msg.header.frame_id = "sonar6"
            range_msg.range = self.range_sensor_data[5] / 1000.0
            self.pub_range6.publish(range_msg)
            rate.sleep()

    def serial_process(self):

        print "in process"
        # n = 0
        while not rospy.is_shutdown():
            if self.ser.read(1) == '\x86':
                if self.ser.read(1) == '\x25':
                    serial_a = self.ser.read(38)
                    # print n
                    # n += 1
                    # print len(serial_a)
                    if len(serial_a) == 38:
                        print "ok"
                        serial_b = struct.unpack('>2H3BH3B12H2B', serial_a)
                        crc = serial_b[-1]
                        crc_split = struct.unpack("37B", serial_a[0:-1])
                        serial_crc = sum(crc_split) & 0xff
                        print serial_crc, crc
                        if serial_crc == crc:
                            # if 1:
                            try:
                                stm32_data = Int32MultiArray()
                                for item in serial_b[0:20]:
                                    stm32_data.data.append(item)
                                self.pub.publish(stm32_data)
                                self.range_sensor_data = stm32_data.data[9:15]
                                print "pub ok"
                            except:
                                print "serial error"
                        else:
                            continue
                    else:
                        continue
                else:
                    continue
            else:
                continue


if __name__ == '__main__':
    try:
        tmp = Serial_32()
        # TODO 线程读取串口
        rospy.spin()
    except rospy.ROSInterruptException:
        pass