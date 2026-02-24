#!/usr/bin/env python3
import sys
import os
# 添加路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rospy
import time
import math
from arm_control import ArmControl
from gripper_control import GripperControlClient
import config
import error_handle as eh