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
from config import config
import error_handle as eh

class TaskSequence:

    def __init__(self):
        rospy.init_node("restaurant_task_sequence")

        self.arm = ArmControl()
        self.gripper = GripperControlClient()

        rospy.loginfo("餐厅3D清洁任务开始")

    # 打印状态
    def print_status(self, step, desc,duration):
        arm_angles = self.arm.get_current_deg()
        gripper_angle = self.gripper.get_current_deg()

        msg = config.STEP_PRINT_FORMAT.format(
            step=step,
            desc=desc,
            duration=duration,
            j1=arm_angles[0],
            j2=arm_angles[1],
            j3=arm_angles[2],
            j4=arm_angles[3],
            j5=arm_angles[4],
            gripper=gripper_angle
        )
        rospy.loginfo(msg)

    # 阶段1：餐具清理
    def run_tableware_stage(self):

        for i in range(config.TABLEWARE_COUNT):

            rospy.loginfo(f"---- 清理餐具 {i+1} ----")

            # 步骤1：夹紧
            rospy.loginfo("步骤1：夹紧餐具，开始执行...")
            target_deg = math.degrees(config.GRIPPER_MAX_ANGLE)

            # 执行动作
            self.gripper.set_angle(target_deg)
            res = eh.check_timeout(expected_duration=1.0)
            rospy.sleep(1.0)  # 耗时1.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "gripper close"):
                return  # 失败 -> 终止整个任务
            self.print_status(1, "夹紧餐具",1.0)

            # 步骤2：抬升 
            rospy.loginfo("步骤2：机械臂抬升，开始执行...")
            raise_deg = [math.degrees(x) for x in config.POUR_RAISE_POSITION]
            ok, msg, idx = eh.check_joint_limits_deg(raise_deg)
            if not ok:
                eh.handle_out_of_range(self.arm, self.gripper, offending_index=idx, offending_value=raise_deg[idx])
                return
            
            # 执行动作
            self.arm.move_to(*raise_deg)
            res = eh.check_timeout(expected_duration=2.0)
            rospy.sleep(2.0)  # 耗时2.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "arm raise"):
                return
            self.print_status(2, "机械臂抬升",2.0)

            # 步骤3：倾倒 
            rospy.loginfo("步骤3：倾倒餐具，开始执行...")

            tilt_deg = [math.degrees(x) for x in config.POUR_TILT_POSITION]
            ok, msg, idx = eh.check_joint_limits_deg(tilt_deg)
            if not ok:
                eh.handle_out_of_range(self.arm, self.gripper, offending_index=idx, offending_value=tilt_deg[idx])
                return
            # 执行动作
            self.arm.move_to(*tilt_deg)
            res = eh.check_timeout(expected_duration=1.0)
            rospy.sleep(1.0)  # 耗时1.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "arm tilt"):
                return
            self.print_status(3, "倾倒餐具",1.0)

            # 步骤4：松开 
            rospy.loginfo("步骤4：松开餐具，开始执行...")

            target_deg = math.degrees(config.GRIPPER_MIN_ANGLE)
            # 执行动作
            self.gripper.set_angle(target_deg)
            res = eh.check_timeout(expected_duration=0.5)
            rospy.sleep(0.5)  # 耗时0.5s
            if not eh.handle_action_result(self.arm, self.gripper, res, "gripper open"):
                return
            self.print_status(4, "松开餐具",0.5)

            # 步骤5：复位 
            rospy.loginfo("步骤5：机械臂复位，开始执行...")
            
            # 执行动作
            self.arm.reset()
            res = eh.check_timeout(expected_duration=2.0)
            rospy.sleep(2.0)  # 耗时2.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "arm reset"):
                return
            self.print_status(5, "机械臂复位",2.0)


    # 阶段2：餐余垃圾清理
    def run_food_stage(self):

        for i in range(config.FOOD_WASTE_COUNT):

            rospy.loginfo(f"---- 清理餐余垃圾 {i+1} ----")

            # 步骤1：机械臂调整至抓取角度
            rospy.loginfo("步骤1：机械臂调整至抓取角度，开始执行...")
            grasp_deg = [math.degrees(x) for x in config.ARM_GRASP_POSITION]
            ok, msg, idx = eh.check_joint_limits_deg(grasp_deg)
            if not ok:
                eh.handle_out_of_range(self.arm, self.gripper, offending_index=idx, offending_value=grasp_deg[idx])
                return
            
            # 执行动作
            self.arm.move_to(*grasp_deg)
            res = eh.check_timeout(expected_duration=2.0)
            rospy.sleep(2.0)  # 耗时2.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "arm grasp"):
                return  # 失败 -> 终止整个任务
            self.print_status(1, "机械臂调整至抓取角度",2.0)

            # 步骤2：爪部闭合 
            rospy.loginfo("步骤2：爪部闭合，开始执行...")
            self.gripper.set_angle(70)
            res = eh.check_timeout(expected_duration=1.0)
            rospy.sleep(1.0)  # 耗时1.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "gripper close"):
                return
            self.print_status(2, "爪部闭合",1.0)

            # 步骤3：抬升 
            rospy.loginfo("步骤3：机械臂抬升，开始执行...")
            raise_deg = [math.degrees(x) for x in config.ARM_LIFT_POSITION]
            ok, msg, idx = eh.check_joint_limits_deg(raise_deg)
            if not ok:
                eh.handle_out_of_range(self.arm, self.gripper, offending_index=idx, offending_value=raise_deg[idx])
                return
            
            # 执行动作
            self.arm.move_to(*raise_deg)
            res = eh.check_timeout(expected_duration=2.0)
            rospy.sleep(2.0)  # 耗时2.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "arm raise"):
                return
            self.print_status(3, "机械臂抬升",2.0)

            # 步骤4：爪部松开 
            rospy.loginfo("步骤4：爪部松开，开始执行...")
            self.gripper.set_angle(0)
            res = eh.check_timeout(expected_duration=0.5)
            rospy.sleep(0.5)  # 耗时0.5s
            if not eh.handle_action_result(self.arm, self.gripper, res, "gripper open"):
                return
            self.print_status(4, "爪部松开",0.5)

            # 步骤5：复位 
            rospy.loginfo("步骤5：机械臂复位，开始执行...")
            
            # 执行动作
            self.arm.reset()
            res = eh.check_timeout(expected_duration=2.0)
            rospy.sleep(2.0)  # 耗时2.0s
            if not eh.handle_action_result(self.arm, self.gripper, res, "arm reset"):
                return
            self.print_status(5, "机械臂复位",2.0)


    # 主流程
    def run(self):
        if config.RUN_TABLEWARE_CLEAN:
            self.run_tableware_stage()

        if config.RUN_FOOD_WASTE:
            self.run_food_stage()

        # 最终复位
        self.arm.reset()
        self.gripper.set_angle(0)

        rospy.loginfo("餐厅3D清洁任务完成，所有动作执行结束")


if __name__ == "__main__":
    try:
        task = TaskSequence()
        task.run()
    except rospy.ROSInterruptException:
        pass