#!/usr/bin/env python3
# src/arm_state_subscriber.py
import sys
import os
# 添加路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import math
import rospy
from sensor_msgs.msg import JointState
from config import config

class ArmStateSubscriber:
    def __init__(self):
        rospy.init_node('arm_state_subscriber', anonymous=False)
        self.prev_positions = None  # 上一帧位置（rad 列表）
        rospy.Subscriber('/arm_joint_angles', JointState, self.callback, queue_size=10)
        rospy.loginfo("[subscriber] 已订阅 /arm_joint_angles")

    def callback(self, msg: JointState):
        # 确保名称顺序正确，且有 5 个关节
        names = msg.name
        positions = list(msg.position)
        # 如果名字不完整或顺序不标准，尝试按 config 中名字映射索引
        if len(positions) != config.ARM_JOINT_NUM:
            rospy.logwarn("[subscriber] 接收到的关节数 != {}, 忽略本帧".format(config.ARM_JOINT_NUM))
            return
        # 计算差值（度）
        if self.prev_positions is None:
            diffs = [0.0]*config.ARM_JOINT_NUM
        else:
            diffs = [math.degrees(positions[i] - self.prev_positions[i]) for i in range(config.ARM_JOINT_NUM)]
        # 打印整齐表格
        lines = []
        degs = [math.degrees(p) for p in positions]
        for i in range(config.ARM_JOINT_NUM):
            name = config.ARM_JOINT_NAMES[i]
            deg = degs[i]
            diff = diffs[i]
            lines.append("{:<6}: {:6.1f}°  差值: {:+5.1f}°".format(name, deg, diff))
        print("\n[arm_state] time: {} ----------------".format(rospy.Time.now().to_sec()))
        for l in lines:
            print(l)
        self.prev_positions = positions

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ArmStateSubscriber()
        node.spin()
    except rospy.ROSInterruptException:
        pass
