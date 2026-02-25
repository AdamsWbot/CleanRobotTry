#!/usr/bin/env python3
import sys
import os
# 添加路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import math
import rospy
from config import config
import time

# 检查机械臂角度（单位：度）是否在 config 限制内。
def check_joint_limits_deg(deg_list):
    
    # 返回 (ok: bool, msg: str, idx:错误序号)
    
    if len(deg_list) != config.ARM_JOINT_NUM:
        return False, f"角度数量错误，期望 {config.ARM_JOINT_NUM}，收到 {len(deg_list)}", None

    for i, name in enumerate(config.ARM_JOINT_NAMES):
        lo_rad, hi_rad = config.ARM_JOINT_LIMITS[name]
        lo_deg = math.degrees(lo_rad)
        hi_deg = math.degrees(hi_rad)
        val = deg_list[i]
        if val < lo_deg or val > hi_deg:
            msg = f"{name} 越界: {val:.1f}° (允许 {lo_deg:.1f}° - {hi_deg:.1f}°)"
            return False, msg, i

    return True, "角度合法", None


def handle_out_of_range(arm, gripper, offending_index=None, offending_value=None):
    """
    处理角度越界：打印信息、复位机械臂、打开爪子（安全），并记录日志。
    该操作视为严重错误，通常调用后应该终止任务。
    """
    rospy.logerr("检测到角度越界! index=%s value=%s", str(offending_index), str(offending_value))
    rospy.loginfo("执行安全复位：机械臂回到 HOME，爪部打开。")
    try:
        arm.reset()
    except Exception as e:
        rospy.logerr("复位机械臂失败: %s", e)
    try:
        # 将爪部打开为安全位（0°）
        gripper.set_angle(math.degrees(config.GRIPPER_MIN_ANGLE))
    except Exception as e:
        rospy.logerr("设置爪部角度失败: %s", e)

    rospy.sleep(getattr(config, "RESET_DURATION", 1.0))
    rospy.logerr("已执行复位，任务应终止或人工干预。")
    return False  # 表示任务需终止/失败

# 检查动作是否超时
def check_timeout(expected_duration):
    """
    返回:
        'ok' 或 'timeout'
    """
    if expected_duration is None or expected_duration <= 0:
        return "ok"

    timeout_threshold = getattr(config, "ACTION_TIMEOUT", 3.0)
    if expected_duration > timeout_threshold:
        rospy.logwarn("预期动作时长 %.2fs 超过超时阈值 %.2fs，标记为 timeout 并跳过。",
                      expected_duration, timeout_threshold)
        return "timeout"

    start = time.time()
    rospy.sleep(expected_duration)
    elapsed = time.time() - start
    rospy.logdebug("等待完成：预期 %.2fs，实际 %.3fs", expected_duration, elapsed)
    return "ok"


def handle_action_result(arm, gripper, action_success, wait_result, info=None):
    """
    根据动作执行结果(action_success: bool)与等待结果(wait_result: 'ok'|'timeout')采取处理。
    - action_success == False : 严重失败 -> 复位爪/臂，返回 False（终止任务）
    - action_success == True and wait_result == 'timeout' : 超时 -> 打警告，返回 True（跳过当前动作，继续）
    - 否则返回 True（正常）
    """
    if not action_success:
        rospy.logerr("动作执行失败: %s", str(info))
        # 尝试安全复位
        try:
            arm.reset()
            open_rad = getattr(config, "GRIPPER_OPEN_ANGLE", None)
            if open_rad is not None:
                open_deg = math.degrees(open_rad)
            else:
                open_deg = math.degrees(getattr(config, "GRIPPER_MIN_ANGLE", 0.0))
            gripper.set_angle(open_deg)
        except Exception as e:
            rospy.logerr("失败处理时异常: %s", e)
        rospy.sleep(getattr(config, "RESET_DURATION", 1.0))
        return False

    # 如果动作成功，但等待结果为 timeout -> 跳过该动作继续
    if wait_result == "timeout":
        rospy.logwarn("动作等待超时，跳过该动作: %s", str(info))
        return True

    # 正常完成
    return True