import math

ARM_JOINT_NUM = 5

ARM_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5"
]

# 关节角度上下限（单位 rad）
ARM_JOINT_LIMITS = {
    "joint1": (0.0, math.radians(270.0)),
    "joint2": (0.0, math.radians(270.0)),
    "joint3": (0.0, math.radians(270.0)),
    "joint4": (0.0, math.radians(270.0)),
    "joint5": (0.0, math.radians(270.0)),
}

# 初始位置（rad）
ARM_INITIAL_POSITION = [
    0.0,
    math.radians(30.0),
    math.radians(60.0),
    math.radians(90.0),
    math.radians(120.0)
]

# 复位位置（rad）
ARM_HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0]


# 机械爪角度范围（rad）
GRIPPER_MIN_ANGLE = 0.0
GRIPPER_MAX_ANGLE = math.radians(90.0)

# 防抖阈值（rad）
GRIPPER_ANTI_SHAKE_THRESHOLD = math.radians(5.0)

# 话题发布频率
PUB_RATE = 1.0


# 项目3

# 机械臂抬升（rad）
POUR_RAISE_POSITION = [
    0.0,
    math.radians(60.0),
    math.radians(90.0),
    0.0,
    0.0
]
POUR_RAISE_DURATION = 2.0  

# 倾倒旋转（rad）
POUR_TILT_POSITION = [
    0.0,
    math.radians(60.0),
    math.radians(90.0),
    math.radians(150.0),
    0.0
]
POUR_TILT_DURATION = 1.0


# 项目4

# 机械臂抓取角度
ARM_GRASP_POSITION = [
    math.radians(30.0),
    math.radians(45.0),
    math.radians(30.0),
    0.0,
    0.0
]

# 机械臂抬升角度
ARM_LIFT_POSITION = [
    math.radians(30.0),
    math.radians(45.0),
    math.radians(30.0),
    math.radians(120.0), 
    0.0
]

# 是否执行餐余垃圾阶段
RUN_FOOD_WASTE = True

# 是否执行餐具清理阶段
RUN_TABLEWARE_CLEAN = True

# 单个动作最大允许时间（秒）
ACTION_TIMEOUT = 3.0


# 餐具数量和餐余垃圾数量（执行次数）
TABLEWARE_COUNT = 2
FOOD_WASTE_COUNT = 2

# 项目4打印格式
STEP_PRINT_FORMAT = (
    "步骤{step}：{desc}完成 共耗时{duration:.1f}秒 "
    "joint1:{j1:.1f}° joint2:{j2:.1f}° joint3:{j3:.1f}° joint4:{j4:.1f}° joint5:{j5:.1f}° "
    "gripper:{gripper:.1f}°"
)

PRINT_FORMAT = "joint{index}: {angle:.3f} rad, 差值: {diff:.3f} rad"
DEBUG_MODE = True