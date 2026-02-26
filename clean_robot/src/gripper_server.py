#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import rospy
from clean_robot.srv import GripperControl, GripperControlResponse

class GripperServer:
    def __init__(self):
        """
        初始化机械爪控制器服务端
        """
        #初始化ROS节点
        rospy.init_node('gripper_server', anonymous=True)
        #初始化上一次有效角度（无有效角度，初始为None）
        self.last_valid_angle = None
        #创建服务 gripper_control
        self.service = rospy.Service('gripper_control', GripperControl, self.handle_gripper_request)
        rospy.loginfo("""
                      柔性机械爪控制服务端已启动
                      服务名称: gripper_control 角度范围:0-90°
                      连续请求角度差值<5°时将拒绝执行
                      """)
        #保持节点运行
        rospy.spin()

    def handle_gripper_request(self, req):
        """
        处理机械爪控制请求
        参数req: GripperControlRequest对象,包含请求的gripper_angle字段
        返回: GripperControlResponse对象,包含响应消息
        """
        #获取请求的机械爪角度
        target_angle = req.gripper_angle

        rospy.loginfo(f"收到机械爪控制请求: 目标角度={target_angle}°")

        #创建响应对象
        response=GripperControlResponse()
        
        
       
       
       
        #角度校验（检查请求的角度是否在有效范围内）
        if target_angle < 0.0 or target_angle > 90.0:
            rospy.logwarn("请求的角度超出有效范围。")

            response.result = False
            if self.last_valid_angle is not None:
                response.current_angle = self.last_valid_angle
            else:
                response.current_angle = 0.0  #默认角度
            response.info = f"角度超出范围，请提供0-90°之间的角度。当前输入角度: {target_angle:.1f}°"
            rospy.loginfo(f"响应: 执行失败-{response.info}")
            return response
        

       
       
       
        
        
        
        
        
        #防抖逻辑校验
        if self.last_valid_angle is not None:
            angle_diff = abs(target_angle - self.last_valid_angle)
            
            if angle_diff < 5.0:
                rospy.logwarn(f"触发防抖保护！角度差值：{angle_diff:.1f}°<5°。")

                response.result = False
                response.current_angle = self.last_valid_angle
                response.info = f"角度变化过小，请提供与上次有效角度至少相差5°的角度。当前输入角度: {target_angle:.1f}° 上次有效角度: {self.last_valid_angle:.1f}°"
                rospy.loginfo(f"响应: 执行失败-{response.info}")
                return response
            
        

        
        
        
        
        
        #角度合法且未触发防抖保护，执行成功
        rospy.loginfo(f"角度合法，执行成功！目标角度: {target_angle:.1f}°")

        #更新上一次有效角度
        self.last_valid_angle = target_angle

        #设置成功响应结果
        response.result = True
        response.current_angle = target_angle 
        response.info = f"角度合法，执行成功！当前机械爪角度: {target_angle:.1f}°"

        #记录当前状态
        rospy.loginfo(f"当前机械爪状态: 角度={target_angle:.1f}°")
        rospy.loginfo(f"响应: 执行成功-{response.info}")

        return response
    





def main():
    try:
        #创建机械爪控制服务端实例
        server=GripperServer()
    except rospy.ROSInterruptException:
        rospy.logerr("机械爪控制服务端已关闭。")
        pass
    except Exception as e:
        rospy.logerr(f"机械爪控制服务端发生异常: {e}")
        pass

if __name__ == '__main__':    
    main()




    