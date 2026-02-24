#!/usr/bin/env python

import rospy
from clean_robot.srv import GripperControl, GripperControlRequest

class GripperControlClient:
    """
    爪子控制客户端类
    职责：封装对机械爪的控制指令，符合项目2考核要求。
    """
    
    def __init__(self):
        # 初始化当前角度变量（初始状态假设为0度）
        self.current_angle = 0.0
        
        # 1. 等待服务端启动
        rospy.loginfo("等待爪子控制服务 /gripper/control 启动...")
        rospy.wait_for_service('/gripper/control')
        self.client = rospy.ServiceProxy('/gripper/control', GripperControl)
        rospy.loginfo("服务连接成功！")

    def set_angle(self, angle):
        """
        设置爪子角度
        参数 angle: 目标角度 (float)
        """
        # --- 1. 角度范围限制 (要求 0-90度) ---
        if angle < 0:
            angle = 0
            rospy.logwarn("警告：目标角度小于0，已修正为0度")
        elif angle > 90:
            angle = 90
            rospy.logwarn("警告：目标角度大于90，已修正为90度")

        # --- 2. 防抖逻辑 (要求：差值小于5度拒绝执行) ---
        angle_diff = abs(angle - self.current_angle)
        if angle_diff < 5.0:
            rospy.loginfo(f"防抖拦截：角度变化太小 ({angle_diff:.1f}度)，不执行动作。")
            return  # 直接返回，不调用服务

        # --- 3. 调用服务 ---
        try:
            # 构造请求
            req = GripperControlRequest()
            req.target_angle = angle
            req.current_angle = self.current_angle  

            # 发送请求并接收响应
            resp = self.client(req)
            
            # --- 4. 更新状态 ---
            if resp.success:
                self.current_angle = angle
                rospy.loginfo(f"成功设置角度: {angle} 度")
            else:
                rospy.logerr(f"服务端执行失败: {resp.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")

    def get_current_deg(self):
        """
        查询当前爪子角度
        返回: 当前角度 (float)
        """
        return self.current_angle
