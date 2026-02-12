#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys


from clean_robot.srv import GripperControl, GripperControlRequest, GripperControlResponse

def gripper_client():
    # 初始化节点，anonymous=True避免节点名冲突
    rospy.init_node('gripper_client', anonymous=True)
    
   
    SERVICE_NAME = '/gripper_control'
    try:
        rospy.wait_for_service(SERVICE_NAME, timeout=10.0)  # 超时10秒
    except rospy.ROSException:
        rospy.logfatal(f"等待服务 {SERVICE_NAME} 超时，请检查服务端是否启动！")
        sys.exit(1)
    
    try:
        # 创建服务代理
        gripper_control = rospy.ServiceProxy(SERVICE_NAME, GripperControl)
        
        # 终端交互提示
        print("=====================================")
        print("柔性机械爪控制客户端 (Client)")
        print("提示：输入 0-90 之间的角度值（0=闭合，90=完全张开）")
        print("输入 'q' 退出程序")
        print("=====================================")
        
        while not rospy.is_shutdown():
            user_input = input("\n请输入目标开合角度: ").strip()
            
            # 1. 处理退出指令
            if user_input.lower() in ['q', 'quit', 'exit']:
                rospy.loginfo("用户主动退出，客户端关闭")
                break
            
            # 2. 处理空输入
            if not user_input:
                print("错误：输入不能为空，请输入0-90之间的数字，或输入 'q' 退出")
                continue
            
            # 3. 转换并校验角度值
            try:
                target_angle = float(user_input)
                # 校验角度范围（0-90°）
                if not (0.0 <= target_angle <= 90.0):
                    print("错误：角度值必须在 0-90 之间，请重新输入")
                    continue
            except ValueError:
                print("错误：请输入有效的数字（如 0、45、90），或输入 'q' 退出")
                continue
            
            # 4. 规范构造服务请求（
            req = GripperControlRequest()
            req.target_angle = target_angle  # 注意：需与你的GripperControl.srv中请求字段名一致
            
            # 5. 调用服务并处理响应
            try:
                response = gripper_control.call(req)  
                
                # 解析并打印响应
                print("\n--- 执行结果反馈 ---")
                if response.result:
                    status = "成功"
                else:
                    status = "失败"
                print(f"执行状态: {status}")
                print(f"当前爪部实际角度: {response.current_angle:.1f}°")
                print(f"详细说明: {response.info}")
                print("-------------------")
                
            except rospy.ServiceException as e:
                rospy.logerr(f"服务调用失败: {str(e)}")
                continue

    except rospy.ROSInterruptException:
        rospy.logwarn("程序被ROS中断信号终止")
    finally:
        # 退出前清理，显式关闭节点
        rospy.signal_shutdown("客户端正常退出")
        sys.exit(0)

if __name__ == "__main__":
    gripper_client()