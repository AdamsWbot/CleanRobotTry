import math

ARM_JOINT_NUM = 5

ARM_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5"
]

# 关节角度上下限（单位 °）
ARM_JOINT_LIMITS = {
    "joint1": (0.0, 270.0),
    "joint2": (0.0, 270.0),
    "joint3": (0.0, 270.0),
    "joint4": (0.0, 270.0),
    "joint5": (0.0, 270.0),
}

# 初始位置
ARM_INITIAL_POSITION = [0.0, 30.0, 60.0, 90.0, 120.0]

# 复位位置
ARM_HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0]


# 机械爪角度范围
GRIPPER_MIN_ANGLE = 0.0        # 完全闭合
GRIPPER_MAX_ANGLE = 90.0       # 完全张开

# 防抖阈值
GRIPPER_ANTI_SHAKE_THRESHOLD = 5.0

# 话题发布频率
PUB_RATE = 1.0


# 项目3

# 机械臂抬升 
POUR_RAISE_POSITION = [0.0, 60.0, 90.0, 0.0, 0.0]
POUR_RAISE_DURATION = 2.0  

# 旋转
POUR_TILT_POSITION = [0.0, 60.0, 90.0, 150.0, 0.0]
POUR_TILT_DURATION = 1.0




PRINT_FORMAT = "joint{index}: {angle}°, 差值: {diff}°"
DEBUG_MODE = True