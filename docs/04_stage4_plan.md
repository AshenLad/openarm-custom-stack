# OpenArm Stage 4：MoveIt2 运动规划与最小 C++ 规划系统

**版本：V2.0 · 2026 年 10 月**

**当前进度：Stage 3 已完成，进入 Stage 4**

**项目定位：机器人系统工程 → 视觉引导操作 → 双臂协调 → Sim-to-Sim**

---

# 一、我的背景与学习目标

我是软件工程硕士，具有以下技术基础：

- Python、PyTorch、计算机视觉
- 语义分割、6D Pose Estimation
- ONNX Runtime、TensorRT、边缘部署
- 机器人视觉与机械臂基础
- 一定的 C++ 使用经验，但希望提高独立工程开发能力

我正在完成个人项目：

`openarm-custom-stack`

项目路径：

```text
/home/ashenlad/code/openarm-custom-stack
```

开发环境：

```text
Ubuntu 24.04
ROS2 Jazzy
OpenArm
MoveIt2 / MTC
ros2_control
Gazebo / MuJoCo（后续阶段）
```

## 1.1 已完成阶段

| 阶段 | 内容 | 状态 |
|---|---|---|
| Stage 0 | Workspace、colcon、开发环境 | 已完成 |
| Stage 1 | Node、Topic、Service、Action | 已完成 |
| Stage 2 | URDF、Xacro、TF2 | 已完成 |
| Stage 3 | ros2_control、轨迹执行 | 已完成 |
| Stage 4 | MoveIt2 运动规划 | 当前 |

Stage 3 学习记录：

```text
docs/03_ros2_control.md
```

已经完成：

- 配置 `mock_components/GenericSystem` Fake Hardware。
- 读取 `/joint_states`。
- 独立编写 Python `FollowJointTrajectory` Action Client。
- 处理 Goal、Feedback 和 Result。
- 实验 Goal Reject、Cancel、Timeout。
- 实验轨迹路径容差失败。
- 理解 Action 结果与客户端退出状态的区别。

当前主要使用左臂：

```text
openarm_left_joint1
...
openarm_left_joint7
```

Planning Group、末端 Link、控制器名称和规划坐标系必须从真实项目配置中确认，不得凭名称猜测。

## 1.2 新版学习原则

**ROS2 框架：够用即可。**

重点不再是熟悉更多 ROS2 API，而是理解机器人运动规划，独立编写 C++ 程序，并定位真实工程问题。

本阶段不深入研究：

- DDS 内部机制
- ROS2 通信中间件源码
- OMPL 源码与规划算法证明
- MoveIt2 的全部 Planning Plugin
- 自定义运动学求解器
- 实时控制器实现
- 复杂任务规划与操作策略

上述知识仅在影响当前任务时补充。

---

# 二、Stage 4 的核心问题

Stage 3 已解决：

> 已经有一条带时间的关节轨迹，如何将它交给机械臂控制器执行？

Stage 4 要解决：

> 已知机械臂当前状态、目标位置和环境约束，如何生成一条可执行、满足约束的运动轨迹？

完整逻辑：

```text
Robot State
    +
Joint / Pose Target
    +
PlanningScene
    +
Constraints
    |
    v
MoveIt2 Planning
    |
    v
Collision-free Path
    |
    v
Time Parameterization
    |
    v
JointTrajectory
    |
    v
Trajectory Execution
    |
    v
ros2_control
    |
    v
Fake Hardware
```

说明：上图为概念上的典型流程。实际规划流水线可能包含规划前后处理适配器，具体是否、如何执行时间参数化，以当前配置为准。

本阶段主要研究：

1. 机器人状态如何表示？
2. 目标关节配置和目标末端位姿有什么不同？
3. MoveIt2 如何检查碰撞和搜索路径？
4. 几何路径如何变为带时间的轨迹？
5. 如何用 C++ 规划和执行？
6. 规划失败如何排查？

---

# 三、本阶段最终验收目标

完成 Stage 4 后，必须能够：

1. 解释 FK、IK、Planning Group、Robot State。
2. 理解 PlanningScene 与碰撞检测。
3. 区分几何路径、轨迹和轨迹执行。
4. 用 C++ 读取当前状态并设置关节目标。
5. 用 C++ 设置末端位姿目标。
6. 独立完成规划、执行和结果检查。
7. 添加、确认及删除环境障碍物。
8. 分析状态、IK、碰撞和轨迹执行相关失败。
9. 将模块作为后续机器人操作系统的基础。

**不以学习了多少 API 为验收标准，以独立实现与调试能力为标准。**

---

# 四、分阶段教学安排

## 4.1 MoveIt2 架构与配置

**学习目标：明确 MoveIt2 究竟使用哪些机器人模型、规划组、规划器及控制器。**

### 必须理解

- `move_group` 的作用
- `MoveGroupInterface` 是什么
- Planning Group 与 Joint Group
- URDF 和 SRDF 的职责
- Robot State
- Planning Frame
- End-Effector Link
- Planning Pipeline
- Trajectory Controller

### 实践任务

启动现有 Fake Hardware 演示环境。

使用 RViz MotionPlanning 面板观察机器人状态与规划结果。

检查项目中的 MoveIt 配置，重点关注：

```text
third_party/openarm_ros2/
  openarm_bimanual_moveit_config/
```

具体文件位置以实际仓库为准。

记录：

```text
Planning Group
Planning Frame
End-Effector Link
Joint Names
Planning Pipeline
Planner ID
Controller Names
```

### 验收标准

能够画出：

```text
My C++ Node / RViz
        |
        v
    move_group
        |
        v
 Planning Pipeline
        |
        v
 Joint Trajectory
        |
        v
 Trajectory Execution
        |
        v
ros2_control Controller
```

能解释每一层的职责。

**止损规则：** 不深入 MoveIt 插件加载、规划器源码和底层 ROS2 通信机制。

---

## 4.2 Robot State、FK、IK 与目标表示

**学习目标：理解机械臂当前位姿与目标位姿之间的数学关系。**

### 必须掌握

**FK：正运动学**

```text
Joint Positions
       |
       v
Forward Kinematics
       |
       v
End-Effector Pose
```

**IK：逆运动学**

```text
Target End-Effector Pose
       |
       v
Inverse Kinematics
       |
       v
Joint Configuration
```

理解：

- 为什么一个 Pose 可能对应多组关节解？
- 为什么部分 Pose 不存在可行 IK 解？
- IK 求解成功是否意味着一定能规划出无碰撞路径？
- 为什么末端位置和姿态需要一起考虑？
- 为什么所有 Pose 都必须具有明确参考坐标系？

### 必要数学知识

复习：

- 旋转矩阵
- 四元数
- 齐次变换矩阵
- SE(3)
- 坐标变换组合

不要求推导完整的七自由度逆运动学求解器。

### 实践任务

在 RViz 中分别完成：

1. 左臂关节空间目标。
2. 左臂末端位姿目标。
3. 一个明显不可达的位姿目标。

记录：

```text
Current Joint State
Current End-Effector Pose
Target Pose
Reference Frame
Planning Result
```

结合 TF2 核对相同坐标系定义下的末端变换。

### 验收标准

能够解释：

> 为什么末端目标位置看起来很接近，但规划仍可能失败？

**后续关联：** Stage 8 的 6D Pose 估计结果需要经过正确的坐标变换，才能成为机器人可使用的目标。

---

## 4.3 运动规划、碰撞检测与轨迹

**学习目标：理解 MoveIt2 如何寻找可行运动，而不是只会调用 `plan()`。**

### 必须理解

- Configuration Space
- Joint-space Planning
- Collision Checking
- Motion Planning
- OMPL
- RRTConnect 基本思想
- Path 与 Trajectory
- Velocity / Acceleration Limit
- Time Parameterization

### RRTConnect 学习要求

理解两棵搜索树如何分别从起点和目标附近扩展，并尝试建立连接。

不要求：

- 自己实现 RRTConnect
- 推导概率完备性
- 阅读 OMPL 源码

### 实践任务

对同一个关节目标：

1. 进行少量重复规划。
2. 观察规划路径是否发生变化。
3. 比较轨迹点和时间信息。
4. 修改速度和加速度缩放参数。
5. 记录规划耗时和轨迹持续时间。

记录：

```text
Planning Success
Planning Time
Trajectory Point Count
Trajectory Duration
Joint Names
```

### 验收标准

能够解释：

- 为什么规划器有时给出不同路径？
- 为什么规划时间不等于轨迹执行时间？
- 为什么找到几何路径后仍可能失败？
- 为什么关节空间运动不保证末端沿直线运动？

不预设修改缩放参数一定只改变时间信息，必须根据真实结果分析。

---

## 4.4 最小 C++ MoveIt2 规划程序

**本阶段最重要的代码任务之一。**

目标：独立编写能够完成关节目标规划的 C++ 程序。

### 工程结构

建立：

```text
src/openarm_moveit_learning/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── minimal_moveit_demo.cpp
└── launch/
    └── learning_demo.launch.py
```

最终结构允许根据实际情况精简。

### 必须理解的 C++ 知识

- 类与对象
- `std::shared_ptr`
- 引用与 const
- 基本异常处理
- CMake 与链接
- ROS2 Executor 的基本作用

### 实现步骤

1. 创建 C++ ROS2 Package。
2. 配置 MoveIt2 依赖。
3. 创建 ROS2 Node。
4. 创建 `MoveGroupInterface`。
5. 确保状态和通信回调能够被处理。
6. 获取当前 Robot State。
7. 设置一个小幅关节目标。
8. 调用 `plan()`。
9. 检查规划错误码。
10. 输出轨迹信息。

### 特别要求

- 不要硬编码未经确认的 Planning Group。
- 不要假设所有机器人使用相同末端 Link。
- 不使用随意填写的关节初值。
- 程序必须确认当前状态有效。
- 不重复启动已经运行的 MoveIt 或控制器节点。
- 接口及调用方式以本机 Jazzy 版本为准。

### 验收标准

程序能够独立输出：

```text
Current State: ...
Target Joints: ...
Planning Result: ...
Planning Time: ...
Trajectory Points: ...
Trajectory Duration: ...
```

**此小节只规划，不执行。**

---

## 4.5 轨迹执行与末端位姿目标

**学习目标：将 Stage 4 与已经掌握的 Stage 3 执行链连接起来。**

### Part A：执行已有规划

在 4.4 程序基础上增加轨迹执行。

需要理解：

- `plan()` 与 `execute()` 的职责。
- 规划成功与执行成功是两个不同结果。
- 执行已保存 Plan 与重新调用 `move()` 的区别。
- 为什么执行前需要检查机器人当前状态。
- 规划起点与实际起点不一致可能造成什么问题。

实践：

```text
Get Current State
        |
        v
Set Joint Target
        |
        v
Plan
        |
        v
Check Planning Result
        |
        v
Execute Saved Plan
        |
        v
Check Execution Result
        |
        v
Read Final State
```

仅在已确认的 Fake Hardware 环境中执行。

### Part B：末端位姿目标

在已有程序基础上：

1. 获取当前末端位姿。
2. 保持当前姿态不变。
3. 沿明确参考坐标系中的一个方向设置小幅位移。
4. 设置 Pose Target。
5. 规划与执行。
6. 读取最终末端位姿。
7. 计算目标与实际位姿误差。

优先使用小幅、可达的目标，避免随意猜测目标 Pose。

### 验收标准

程序同时支持：

```text
Joint Target
Pose Target
```

并且分别检查规划与执行结果。

最终能够解释：

```text
C++ MoveGroupInterface
        |
        v
MoveIt Planning
        |
        v
Trajectory Execution
        |
        v
FollowJointTrajectory
        |
        v
JointTrajectoryController
        |
        v
Fake Hardware
```

这部分完成后，不再额外深入 ROS2 Action 相关理论。

---

## 4.6 PlanningScene 与环境障碍物

**学习目标：掌握后续视觉抓取和双臂避碰必需的场景管理能力。**

### 必须理解

- PlanningScene
- CollisionObject
- Self Collision
- Environment Collision
- Collision Geometry
- PlanningSceneInterface

明确区分：

```text
RViz Visual Object
MoveIt Collision Object
Gazebo Physical Object
```

它们并不天然等价。

### 实践任务

创建：

```text
scene_obstacle_demo.cpp
```

实现：

1. 在机器人工作空间内添加盒状障碍物。
2. 确认障碍物已进入 PlanningScene。
3. 对相同目标重新规划。
4. 观察规划结果。
5. 删除障碍物。
6. 确认删除操作已生效。
7. 再次进行规划。

记录：

```text
Obstacle ID
Obstacle Pose
Obstacle Size
Obstacle Frame
Planning Result
```

避免在测试开始时让障碍物覆盖机器人当前配置，防止将起始状态碰撞误判为路径搜索失败。

### 验收标准

能够独立操作环境 CollisionObject，并解释障碍物为什么影响规划。

不要求每次加入障碍物都必须出现绕行路径；也可能规划失败或保持原路径。

### 与后续项目的关联

Stage 5：

```text
Pick Object
    |
    v
Attach Object
    |
    v
Modify PlanningScene
```

Stage 8：

```text
6D Pose Estimation
    |
    v
Object Pose
    |
    v
PlanningScene Update
```

本阶段不提前深入 Attach、Detach 和 Allowed Collision Matrix，留给真正的操作任务阶段。

---

## 4.7 故障诊断、基础性能记录与阶段验收

**学习目标：从会调用 API 进阶到能够独立排查机器人规划系统问题。**

### 必须完成的三个故障实验

**案例一：机器人状态异常**

重点检查：

```text
/joint_states
Robot State Completeness
State Timestamp
ROS Time
Executor
```

**案例二：末端位姿目标规划失败**

重点检查：

```text
Target Frame
End-Effector Link
Quaternion
IK
Joint Limits
Collision
```

**案例三：加入障碍物后规划失败**

重点检查：

```text
PlanningScene Update
Start State Collision
Goal State Collision
Path Collision
Planning Timeout
```

### 故障记录模板

每个案例都要求记录：

```text
问题描述
输入条件
错误码
关键日志
故障所在层
诊断依据
修复方式
最终结果
```

不能把所有失败都简单归因于 IK、RRT 或机械臂不可达。

### 增加最小性能记录

为了后续 Sim-to-Sim 实验，Stage 4 就开始记录统一格式的基础数据。

建议至少包含：

```text
target_type
start_state
target
planning_success
planning_time
trajectory_duration
trajectory_points
execution_success
final_joint_error
final_ee_position_error
final_ee_orientation_error
```

其中：

- 关节误差必须保证关节名称和顺序对应。
- 末端位姿误差必须统一坐标系及 TCP 定义。
- 应区分最终状态采样误差、轨迹跟踪误差和目标误差。
- Fake Hardware 测得的结果不能代表真实动力学或接触性能。

当前只做简单数据记录，不开发复杂 Benchmark 框架。

### 验收标准

独立完成三个故障案例，能够解释故障归属及验证方法。

能够用自己的话完整解释：

```text
Stage 3:
Trajectory → Controller → Hardware

Stage 4:
Target → Planning → Trajectory → Controller

Stage 5:
Task → MTC Stages → Motion Planning → Execution
```

至此完成 Stage 4。

---

# 五、选修内容

以下内容不作为 Stage 4 必过条件。

## 5.1 笛卡尔路径

了解：

- Joint-space Path 与 Cartesian Path 的区别
- `computeCartesianPath`
- 笛卡尔路径插值
- 路径完成比例与有效性检查

实验：

给定两个距离较近的末端位姿，尝试生成笛卡尔路径，并检查结果。

没有完成全部预期路径时，不直接执行部分路径作为完整任务。

该知识对 Stage 5 的 Approach、Lift 和 Stage 9 的物体对齐有帮助。

## 5.2 双臂规划初步观察

使用已有双臂模型：

- 查看左臂和右臂 Planning Group。
- 理解双臂组合规划组是否存在。
- 观察单臂规划与双臂协调规划的区别。

不在 Stage 4 实现复杂双臂抓取。

真正的双臂协调与物体对齐留给后续阶段。

---

# 六、最终工程交付物

## 6.1 代码

```text
openarm-custom-stack/
├── src/
│   └── openarm_moveit_learning/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── src/
│       │   ├── minimal_moveit_demo.cpp
│       │   └── scene_obstacle_demo.cpp
│       └── launch/
│           └── learning_demo.launch.py
└── docs/
    └── 04_moveit_basics.md
```

路径和结构可以根据真实项目调整。

核心要求是：

- 代码可编译。
- 有明确运行方法。
- 能重复执行实验。
- 有错误处理。
- 不依赖未经说明的手动配置。
- 不破坏原有 OpenArm 功能。

## 6.2 学习文档

`docs/04_moveit_basics.md` 必须包含：

1. MoveIt2 规划与控制架构。
2. 实际 Planning Group 和模型配置。
3. FK、IK、PlanningScene 的理解。
4. 关节目标与末端位姿目标实验。
5. C++ 规划与执行结果。
6. 碰撞障碍物实验。
7. 三种规划故障案例。
8. 基础性能记录。
9. Fake Hardware 的能力边界。
10. 自己对 Stage 4 的总结。

## 6.3 演示结果

至少保留：

- 一次关节目标规划与执行。
- 一次末端位姿目标规划与执行。
- 一次添加障碍物后的规划实验。
- 一次能够解释的规划失败案例。

---

# 七、Stage 4 面试检查

完成后必须能够独立回答：

1. FK 和 IK 有什么区别？为什么 IK 有解不代表路径可行？
2. MoveIt2 的 PlanningScene 具体解决什么问题？
3. OMPL 和 RRTConnect 分别是什么？
4. MoveIt2 如何从目标关节状态生成可执行轨迹？
5. 几何路径与带时间信息的轨迹有什么区别？
6. `plan()` 成功以后，`execute()` 为什么仍然可能失败？
7. 如何确认规划场景已经添加了障碍物？
8. 当前状态读取失败会影响哪些规划步骤？
9. 6D Pose 网络输出的目标位姿，如何进一步变成 MoveIt2 的运动目标？
10. 为什么同一条规划轨迹在 Fake Hardware、Gazebo 和 MuJoCo 中可能表现不同？

最后两题只要求能够建立基本逻辑，不提前深入后续阶段。

---

# 八、Stage 4 的学习边界

**必须深入掌握：**

- FK / IK 概念
- 坐标系与 SE(3)
- 关节目标与位姿目标
- PlanningScene
- 规划、轨迹与执行
- C++ 规划程序
- 失败诊断

**工程够用即可：**

- MoveGroupInterface API
- MoveIt 参数与插件配置
- ROS2 Executor
- 规划器选择和配置
- CMake / Launch

**明确留到后续：**

- MTC 多阶段抓放
- Attach / Detach
- 双臂协调操作
- 6D Pose 感知融合
- 视觉伺服
- Gazebo 物理仿真
- MuJoCo 动力学
- Sim-to-Sim
- VLA / RL

---

# 九、学习时间与止损原则

建议 Stage 4 按约 **3–5 周、累计 15–25 小时**进行初步规划，具体以工作安排和真实调试难度为准，不为了赶进度跳过验收。

如果某个问题耗时过长：

1. 先定位属于配置、代码、通信、规划还是控制层。
2. 尝试构造最小复现。
3. 查询当前 Jazzy 安装环境中的实际 API。
4. 保留错误记录。
5. 不为研究无关源码无限延期。

核心目标是尽快形成可用的 MoveIt2 规划能力，然后进入 Stage 5。

---

# 十、给新 Session 的教学协议

你是我的 **ROS2 / MoveIt2 / C++ 机器人系统工程导师**。

请严格遵守以下要求：

1. 首先阅读项目中的 `docs/03_ros2_control.md`，了解已经完成的 Stage 3。
2. 确认现有 ROS2 环境、Planning Group、控制器和启动方式。
3. 只从 Stage 4.1 开始，不要一次性教授完整 Stage。
4. 每次只介绍一个小知识单元，安排一个对应实验。
5. 先讲为什么需要这个知识，再讲原理，最后讲 API。
6. 对新的数学或机器人术语，给出必要的公式、直观解释与例子。
7. 我需要独立敲写 C++ 代码，不要直接生成完整解决方案。
8. 允许提供函数签名、必要接口说明和测试要求，由我完成实现。
9. 检查我的代码时，严格指出编译问题、逻辑错误、边界条件和潜在运行时风险。
10. 优先使用本机安装的 Jazzy API，不能盲目照搬其他 ROS2 版本的教程。
11. 遇到日志时先分析故障在哪一层，再考虑修复命令。
12. 每次实验都要让我解释输入、输出、调用关系和实际观察结果。
13. 不要为了丰富课程而扩展无关 ROS2 理论。
14. 不提前教授 Stage 5 的 MTC、Stage 8 的 6D Pose 或后续 Sim-to-Sim。
15. 学完一个小节后给出验收问题，确认真正理解后才能进入下一节。

**教学目标不是让我成为 MoveIt2 API 专家，而是让我能够独立开发、调试和解释一个 C++ 机器人运动规划模块。**

## 新 Session 的第一条任务

请先检查：

```text
docs/03_ros2_control.md
```

结合我的实际 OpenArm 项目，确认当前 Fake Hardware 与 MoveIt2 配置。

然后只开始：

**Stage 4.1：MoveIt2 架构与实际配置。**

本次先不要生成 C++ 规划程序，也不要进入 MTC 或 Gazebo。