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

# 执行代码，并判断是否成功，是否超时
def perform_action_with_timeout(action_fn, args=None, expected_duration=0.0):
    """
    返回：
        'ok'      -> 执行成功并在允许时长内等待完成
        'failed'  -> action_fn 返回 False（调用失败）
        'timeout' -> expected_duration > config.ACTION_TIMEOUT（视为超时，应跳过）
    """
    if args is None:
        args = ()
    try:
        ok = action_fn(*args)
    except Exception as e:
        rospy.logerr("执行动作时抛异常: %s", e)
        return "failed"

    if not ok:
        rospy.logerr("动作函数返回失败 (False)")
        return "failed"

    # 如果预期时长超过最大时长 -> 视为超时并跳过
    if expected_duration is not None and expected_duration > config.ACTION_TIMEOUT:
        rospy.logwarn(
            "预期动作时长 %.2fs 超过超时阈值 %.2fs，标记为 timeout 并跳过。",
            expected_duration, config.ACTION_TIMEOUT
        )
        return "timeout"

    if expected_duration and expected_duration > 0:
        start = time.time()
        rospy.sleep(expected_duration)
        elapsed = time.time() - start
    return "ok"


# 根据 perform_action_with_timeout 的结果采取处理
def handle_action_result(arm, gripper, result_type, info=None):
    """
    - 'ok' -> 返回 True，继续下一个步骤
    - 'timeout' -> 打印警告，返回 True（跳过当前动作，继续后续动作）
    - 'failed' -> 记录并复位, 返回 False
    """
    if result_type == "ok":
        return True
    elif result_type == "timeout":
        rospy.logwarn("动作超时，跳过该动作: %s", str(info))
        return True  # 跳过，继续后续动作
    else:  # failed
        rospy.logerr("动作执行失败，执行安全处理: %s", str(info))
        # 尝试复位
        try:
            arm.reset()
            gripper.set_angle(math.degrees(config.GRIPPER_MIN_ANGLE))
        except Exception as e:
            rospy.logerr("失败处理时异常: %s", e)
        rospy.sleep(2.0)  # 等待复位完成
        return False