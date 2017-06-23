#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import string
import re
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String


class Serial_32:
    def __init__(self):
        rospy.init_node('serial_fuse', anonymous=True)
        self.pub = rospy.Publisher('stm32_data', Int32MultiArray, queue_size=100)
        rospy.Subscriber("charging", String, self.charge_Callback)
        self.ser = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)
        self.ser.flushOutput()
        self.ser.flushInput()
        t1 = threading.Thread(target=self.serial_process)
        t1.start()

    def charge_Callback(self, msg):
        print msg.data
        data = '\x00'
        self.ser.write(data)

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
                        serial_b = struct.unpack('<2H3BH3B12H2B', serial_a)
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