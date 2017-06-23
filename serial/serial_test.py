#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import string
import re


class Serial_Motor:
    def __init__(self):
        self.ser = serial.Serial(port='/dev/pts/11', baudrate=9600, timeout=1)
        self.ser.flushOutput()
        self.ser.flushInput()

    def reset(self):
        buffer = bytearray(b'\x23\x00\x00\x00\x00\x00')
        self.ser.write(buffer)
        print map(hex, buffer)

    def stop(self):
        buffer = bytearray(b'\x23\x02\x00\x00\x00\x00')
        self.ser.write(buffer)
        print map(hex, buffer)

    def point_move(self, direction, position):
        buffer = bytearray(b'\x23\x01')
        data_buffer = struct.pack(">4B", 0x00, direction, 0x00, position)
        buffer += data_buffer
        self.ser.write(buffer)
        print map(hex, buffer)

    def append_sum(self, buffer):
        checksum = sum(buffer[2:])
        checksum &= 0xFF  # 强制截断
        buffer.append(checksum)
        return buffer


if __name__ == '__main__':
    try:
        # rospy.init_node('serial_fuse', anonymous=True)
        tmp = Serial_Motor()
        while 1:
            str = raw_input("Please enter: ")
            print str
            if str == 'stop':
                tmp.stop()
            elif str == 'reset':
                tmp.reset()
            else:
                str_split = string.split(str, ' ')
                tmp.point_move(string.atoi(str_split[0]), string.atoi(str_split[1]))
        # TODO 线程读取串口
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass