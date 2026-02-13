#!/usr/bin/env python3
# src/arm_joint_publisher.py
import sys
import os
# 添加路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import threading
import time
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from config import config


"""
发布节点:arm_joint_publisher
- 话题：/arm_joint_angles
- 消息类型:sensor_msgs/JointState
- 支持在终端通过交互修改角度（单位：度）
- 角度输入将做 0-270° 检查
"""

class ArmJointPublisher:
    def __init__(self):
        rospy.init_node('arm_joint_publisher', anonymous=False)
        self.pub = rospy.Publisher('/arm_joint_angles', JointState, queue_size=10)
        self.rate = rospy.Rate(config.PUB_RATE)
        # 当前角度（rad）
        self.positions = list(config.ARM_INITIAL_POSITION)
        self.lock = threading.Lock()
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()
        rospy.loginfo("[publisher] 启动，发布频率: {} Hz".format(config.PUB_RATE))

    def _validate_rad(self, joint_index, rad_value):
        name = config.ARM_JOINT_NAMES[joint_index]
        lo, hi = config.ARM_JOINT_LIMITS[name]
        return lo <= rad_value <= hi

    def _input_loop(self):
        while not rospy.is_shutdown():
            try:
                s = input("publisher> ").strip()
            except EOFError:
                break
            if not s:
                continue
            parts = s.split()
            cmd = parts[0].lower()
            if cmd == 'set':
                if len(parts) < 3:
                    print("格式错误，参考示例： set 1 45 或 set all d1 d2 d3 d4 d5")
                    continue
                target = parts[1].lower()
                if target == 'all':
                    vals = parts[2:]
                    if len(vals) != config.ARM_JOINT_NUM:
                        print("需要 {} 个角度值，单位°".format(config.ARM_JOINT_NUM))
                        continue
                    try:
                        degs = [float(v) for v in vals]
                    except:
                        print("角度解析出错，请输入数字")
                        continue
                    rads = [math.radians(d) for d in degs]
                    ok = True
                    for i, r in enumerate(rads):
                        if not self._validate_rad(i, r):
                            print("角度超出范围：{} = {:.1f}°（允许 0-270°）".format(config.ARM_JOINT_NAMES[i], math.degrees(r)))
                            ok = False
                    if not ok:
                        print("设置失败：至少有一项超出范围")
                        continue
                    with self.lock:
                        self.positions = rads
                    print("全部关节已更新")
                else:
                    # set single joint
                    try:
                        idx = int(target) - 1  # user uses 1..5
                        if idx < 0 or idx >= config.ARM_JOINT_NUM:
                            print("关节编号必须是 1-{}".format(config.ARM_JOINT_NUM))
                            continue
                        val = float(parts[2])
                    except:
                        print("参数解析错误，示例： set 1 45")
                        continue
                    rad = math.radians(val)
                    if not self._validate_rad(idx, rad):
                        print("角度超出范围（允许 0-270°），设置被拒绝")
                        continue
                    with self.lock:
                        self.positions[idx] = rad
                    print("{} 已设置为 {:.1f}°".format(config.ARM_JOINT_NAMES[idx], val))
            elif cmd == 'reset':
                with self.lock:
                    self.positions = [0.0]*config.ARM_JOINT_NUM
                print("已复位到 0°")
            else:
                print("未知命令。可用命令：set, set all, reset")
    def run(self):
        while not rospy.is_shutdown():
            js = JointState()
            js.header = Header()
            js.header.stamp = rospy.Time.now()
            js.name = list(config.ARM_JOINT_NAMES)
            with self.lock:
                js.position = list(self.positions)
            self.pub.publish(js)
            # debug
            if config.DEBUG_MODE:
                degs = [math.degrees(x) for x in js.position]
                rospy.logdebug("[publisher] positions deg: {}".format(", ".join("{:.1f}".format(d) for d in degs)))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ArmJointPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass