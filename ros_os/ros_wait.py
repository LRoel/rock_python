#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import os
import time


time.sleep(5)
os.system('gnome-terminal -x bash -c "roslaunch af_robot function_demo.launch"')