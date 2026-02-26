*本功能输入与输出统一使用角度制*
*本功能所有项目可通过 `roslaunch clean_robot project(1/2/3/4)` 一键启动*

# 机器人清洁挑战赛微缩考核
开发环境：Ubuntu20.04 + ROS-Noetic


## 项目1
###  功能说明
- **核心功能**：实现5轴机械臂的关节角度发布与状态订阅。  
- **通信机制**：  
  - **话题 (Topic)**：`/joint_states`  
  - **消息类型 (Msg)**：`sensor_msgs/JointState`  
- **参数配置 (源自 `config/config.py`)**：  
  - **关节数量**：5轴。  
  - **关节范围**：0° ~ 270° (0.0 ~ 4.712 rad)。  
  - **初始位置**：`[0°, 30°, 60°, 90°, 120°]`。  
  - **复位位置**：`[0°, 0°, 0°, 0°, 0°]`。  

### 运行步骤

.  确保已编译并激活环境。
.  打开终端，运行：  
    `roslaunch clean_robot project1.launch`
    
    

发布节点支持手动终端输入修改关节角度，5轴机械臂关节角度取值**0-270°**  
输入示例：
输入|效果
---|---
set 1 45|将 joint1 设为 45°
set all 20 50 60 90 210|按序一次设置所有关节（单位°）
reset|复位所有关节到 0°

*因要求实时打印关节状态，终端输出频繁，建议输入采用复制粘贴*  

---

## 项目2
此模块包含一个服务端（处理请求）和一个客户端（发送指令），通过服务通信控制机械爪开合。



**运行指令**：
`roslaunch clean_robot project2.launch`

**输出示例**：
*   **服务端输出**：
    `[INFO] 接收到请求，目标角度: 50.00°`
    `[WARN] 角度差值过小，无需动作`
*   **客户端交互**：
    `请输入目标角度 (0-90): 50(单位°)`
    `[RESULT] 执行成功! 当前爪部角度: 50.00°`

---
## 项目3
复刻餐厅赛项核心动作：夹取 -> 抬升 -> 倾倒 -> 收纳。严格遵循赛事时序与参数。

### 动作流程
1.  **夹取**：机械爪闭合（90°），夹紧餐具。
2.  **抬升**：机械臂关节抬升至指定高度（joint2=60°, joint3=90°）。
3.  **倾倒**：机械臂关节旋转倾倒（joint4=150°），耗时 1 秒。
4.  **收纳**：机械爪松开（0°），机械臂复位。

**运行指令**：  
`roslaunch clean_robot project3.launch`  

**输出示例**：
*   **程序启动**：
    `[INFO] 步骤1/6: 爪部闭合中...`
*   **动作执行**：
    `[INFO] 步骤3/6: 旋转倾倒中... joint4=150°`
*   **完成提示**：
    `[INFO] 所有动作执行完毕，程序退出。`
---

## 项目4


### 功能说明

* 自动完成：两处餐具清理（循环 `TABLEWARE_COUNT` 次，可通过设置`RUN_TABLEWARE_CLEAN`） + 两处餐余垃圾清理（循环 `FOOD_WASTE_COUNT` 次，可通过设置 `RUN_FOOD_WASTE` 关闭）
* 每一步按照模式执行：**越界检查 → 执行动作 → 等待期望时长 → 统一异常处理 → 打印状态**
* 使用统一配置（`config.py`）管理所有位置、时长与阈值，便于复现与调参

### 话题与服务

* 话题

  * `/arm_joint_angles` — `sensor_msgs/JointState`

    * `header`（stamp）
    * `name[]`（string） — 使用 `config.ARM_JOINT_NAMES`
    * `position[]`（float64） — 单位 **rad**

* 服务

  * `/gripper_control` — `GripperControl.srv`

    * 请求：`float32 gripper_angle`（单位：**度**）
    * 响应：`bool result`, `float32 current_angle`, `string info`

### 运行步骤:

1.  确保工作空间已编译（catkin_make）并激活环境（source devel/setup.bash）。
2.  打开终端，运行以下命令启动项目：  
    `roslaunch clean_robot project4.launch`

---

## 异常处理说明

1. **角度越界**

   * 检测点：在任何会发布到机械臂的动作前（主节点）调用 `error_handle.check_joint_limits_deg()`。
   * 处理策略：调用 `error_handle.handle_out_of_range()` → 执行 `arm.reset()`、将爪子安全复位，并将任务视为失败/终止（返回 False）。

2. **动作失败（动作函数返回 False 或抛异常）**

   * 例如 `arm.move_to()` 或 `gripper.set_angle()` 返回 False（非法输入、服务失败等）。
   * 处理策略：在 `handle_action_result()` 中捕获，尝试安全复位并终止任务（返回 False）。

3. **动作超时**

   * 判断：`check_timeout(expected_duration)` 会比较 `expected_duration` 与 `config.ACTION_TIMEOUT`。若预期时长 > 超时阈值，则返回 `'timeout'`。
   * 处理策略：超时时打印警告并**跳过该动作继续后续步骤**（减少单点失败导致整个序列中断的风险）。  
  
4. **爪部防抖（服务端约束）**

   * 服务端拒绝与上次有效角度差 < 5° 的请求，返回 `result = False` 并在 `info` 中说明。主节点收到后会按动作失败规则处理（复位或终止）。


---

## 模块注释

* `config.py`

  * 所有参数集中管理（单位：内部使用 **rad**，外部接口/打印使用 **°**）。
  * 典型条目：`ARM_JOINT_NAMES`, `ARM_JOINT_LIMITS`, `ARM_INITIAL_POSITION`, `POUR_RAISE_POSITION`, `POUR_TILT_POSITION`, `GRIPPER_CLOSE_ANGLE`, `ACTION_TIMEOUT`, `TABLEWARE_COUNT`, `FOOD_WASTE_COUNT` 等。

* `arm_control.py`

  * 类 `ArmControl`（**模块**，非节点）
  * 函数：`move_to(j1,j2,j3,j4,j5)`（输入/输出均为 **度**，内部发布 `sensor_msgs/JointState` 使用 **弧度**）、`reset()`、`get_current_deg()`
  * 责任：校验角度（使用 config 的限制）、构建并发布 `JointState`（topic `/arm_joint_angles`）

* `gripper_server.py`

  * 服务端（节点），实现 `/gripper_control` srv
  * srv 定义（请求/响应）：

    ```text
    # 请求
    float32 gripper_angle
    ---
    # 响应
    bool result
    float32 current_angle
    string info
    ```
  * 逻辑：角度合法性检查（0–90°），防抖：若与上次有效角度差值 < 5° 则拒绝；返回执行结果并记录实际角度/说明

* `gripper_control.py`

  * Gripper 客户端模块
  * 封装 `set_angle(angle)` 与 `get_current_deg()`，处理 `rospy.ServiceException` 与响应解析

* `error_handle.py`

  * 异常处理模块
  * 函数：`check_joint_limits_deg(deg_list)`（返回 ok/msg/index）、`check_timeout(expected_duration)`（等待并判断是否超时，返回 `'ok'` 或 `'timeout'`）、`handle_action_result(...)`（根据执行/等待结果统一处理）、`handle_out_of_range(...)`（处理角度越界）



