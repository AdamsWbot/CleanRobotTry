#!/usr/bin/env python3
import sys
import os
# 添加路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from config import config

# 仅为机械臂控制模块而非节点
class ArmControl:
    def __init__(self):
        self.pub = rospy.Publisher('/arm_joint_angles', JointState, queue_size=10)
        self.joint_names = config.ARM_JOINT_NAMES
        self.joint_num = config.ARM_JOINT_NUM
        self.current_positions = list(config.ARM_INITIAL_POSITION)

        rospy.loginfo("[ArmControl] 初始化完成")

    # 校验输入角度（单位°）
    def _validate_angles(self, deg_list):
        if len(deg_list) != self.joint_num:
            rospy.logerr("输入角度数量错误！")
            return False
        for i, joint_name in enumerate(self.joint_names):
            min_rad, max_rad = config.ARM_JOINT_LIMITS[joint_name]

            # 转换为度进行比较
            min_deg = math.degrees(min_rad)
            max_deg = math.degrees(max_rad)

            if deg_list[i] < min_deg or deg_list[i] > max_deg:
                rospy.logerr(
                    f"{joint_name} 角度超出范围！"
                    f" 输入:{deg_list[i]:.1f}° "
                    f" 允许范围:{min_deg:.1f}° - {max_deg:.1f}°"
                )
                return False
        return True

    # 发布JointState（rad）
    def _publish(self, rad_list):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = rad_list

        self.pub.publish(msg)
        
    # 移动至指定角度（单位°），返回True/False
    def move_to(self, j1, j2, j3, j4, j5):
        deg_list = [j1, j2, j3, j4, j5]
        # 角度校验
        if not self._validate_angles(deg_list):
            rospy.logwarn("move_to 执行失败：角度非法")
            return False

        # 转换为rad
        rad_list = [math.radians(d) for d in deg_list]

        # 发布
        self._publish(rad_list)

        # 更新内部状态
        self.current_positions = rad_list

        rospy.loginfo(
            "move_to 执行成功: " +
            ", ".join(f"{d:.1f}°" for d in deg_list)
        )

        return True

    # 全部角度复位至0°
    def reset(self):
        rospy.loginfo("机械臂复位到0°")
        return self.move_to(0, 0, 0, 0, 0)

    # 返回当前角度（单位°）
    def get_current_deg(self):

        return [math.degrees(rad) for rad in self.current_positions]