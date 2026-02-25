#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
# 让Python能找到同一目录下的模块（因为我们将三个.py都放在src/下）
sys.path.append(os.path.dirname(__file__))

from arm_control import ArmControl
from gripper_control import GripperControlClient

def emergency_reset(arm, gripper):
    """紧急复位：关节归零、爪部松开，并终止程序"""
    rospy.logerr("检测到异常，执行紧急复位")
    try:
        arm.reset()
        gripper.set_angle(0)
        rospy.sleep(1)  # 等待复位完成
    except Exception as e:
        rospy.logerr(f"复位过程中发生异常: {e}")
    finally:
        raise SystemExit(1)  # 直接退出

def main():
    # 初始化ROS节点（固定名称，便于调试）
    rospy.init_node('restaurant_arm_pour', anonymous=False)

    # 创建机械臂和爪部控制对象
    arm = ArmControl()
    gripper = GripperControlClient()

    try:
        # ---------- 步骤1：爪部闭合至90° ----------
        rospy.loginfo("步骤1：爪部闭合至90°")
        if not gripper.set_angle(90):
            raise RuntimeError("爪部设置角度失败")
        current_gripper = gripper.get_current_deg()
        rospy.loginfo("步骤1：当前爪部角度 %.1f°", current_gripper)
        rospy.sleep(1.0)

        # ---------- 步骤2：机械臂抬升 ----------
        rospy.loginfo("步骤2：机械臂抬升中")
        if not arm.move_to(0, 60, 90, 0, 0):
            raise RuntimeError("机械臂抬升失败")
        arm_deg = arm.get_current_deg()
        rospy.loginfo("步骤2：joint2当前角度 %.1f°，joint3当前角度 %.1f°",
                      arm_deg[1], arm_deg[2])
        rospy.sleep(2.0)

        # ---------- 步骤3：机械臂旋转至倾倒角度 ----------
        rospy.loginfo("步骤3：机械臂旋转至倾倒角度")
        if not arm.move_to(0, 60, 90, 150, 0):
            raise RuntimeError("机械臂旋转失败")
        arm_deg = arm.get_current_deg()
        rospy.loginfo("步骤3：joint4当前角度 %.1f°", arm_deg[3])
        rospy.sleep(1.0)  # 耗时1s

        # ---------- 步骤4：爪部松开至0° ----------
        rospy.loginfo("步骤4：爪部松开至0°")
        if not gripper.set_angle(0):
            raise RuntimeError("爪部松开失败")
        current_gripper = gripper.get_current_deg()
        rospy.loginfo("步骤4：当前爪部角度 %.1f°", current_gripper)
        rospy.sleep(0.5)  # 耗时0.5s

        # ---------- 步骤5：机械臂复位 ----------
        rospy.loginfo("步骤5：机械臂复位")
        if not arm.reset():
            raise RuntimeError("机械臂复位失败")
        rospy.loginfo("步骤5：所有关节已复位至0°")
        rospy.sleep(2.0)

        # ---------- 步骤6：爪部保持0° ----------
        rospy.loginfo("步骤6：爪部保持0°，动作结束")
        rospy.loginfo("餐厅倾倒+收纳动作完成")

    except Exception as e:
        rospy.logerr(f"执行过程中发生错误: {e}")
        emergency_reset(arm, gripper)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass