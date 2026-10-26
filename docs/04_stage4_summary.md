# OpenArm 机器人学习项目：Stage 4 完结总结与 Stage 5 交接

**完成日期：2026-10-26**

**当前状态：Stage 0～4 已完成，下一阶段 Stage 5：MoveIt Task Constructor（MTC）。**

## 一、项目背景与开发环境

项目目标：构建自己的机器人软件栈，逐步掌握 ROS2、MoveIt2、机器人运动规划、任务规划和仿真技术，之后进一步开展机器人视觉、6D Pose、模仿学习、强化学习、Sim-to-Real 等研究。

项目地址：

https://github.com/AshenLad/openarm-custom-stack

本地路径：

```text
/home/ashenlad/code/openarm-custom-stack
```

开发环境：

- Ubuntu 24.04 / ROS2 Jazzy
- Distrobox 容器环境，宿主系统 CachyOS
- C++17、rclcpp、ament_cmake、colcon
- MoveIt2、OMPL、ros2_control
- Fake Hardware：`mock_components/GenericSystem`

当前机器人与规划配置：

| 配置 | 值 |
|---|---|
| Robot Model | `openarm_v20` |
| Planning Group | `left_arm` |
| 左臂自由度 | 7 |
| Planning Frame | `world` |
| End-Effector Link | `openarm_left_ee_base_link` |
| Planning Pipeline | `ompl` |
| Planner | `geometric::RRTConnect` |
| Trajectory Controller | `left_joint_trajectory_controller` |

**安全要求：当前所有运动执行都必须限制在已经确认的 Fake Hardware 环境。**

## 二、Stage 4 完成情况

### 4.1 MoveIt2 架构与配置——完成

掌握了：

- `move_group` 与 `MoveGroupInterface` 的职责区别
- URDF、SRDF、Planning Group、RobotState
- Planning Pipeline 与 OMPL
- MoveIt2 和 ros2_control 的连接关系
- 控制器与 FollowJointTrajectory Action

能够解释完整的软件调用链：

```text
C++ MoveGroupInterface
        ↓
     move_group
        ↓
   OMPL / RRTConnect
        ↓
    RobotTrajectory
        ↓
Trajectory Execution Manager
        ↓
  FollowJointTrajectory
        ↓
JointTrajectoryController
        ↓
    GenericSystem
        ↓
    /joint_states
```

### 4.2 RobotState、FK、IK 与目标表示——核心内容完成

理解并实践了：

- 关节空间配置与末端空间位姿的关系
- 正运动学 FK：关节角度 → 末端位姿
- 逆运动学 IK：末端目标位姿 → 可行关节配置
- 齐次变换矩阵、旋转矩阵、四元数、参考坐标系
- Joint Target 与 Pose Target 的区别
- 为什么 IK 有解不保证无碰撞路径可行

通过 `RobotState::getGlobalLinkTransform()`，使用规划轨迹终点的关节角度重新计算末端位置，对规划结果进行了 FK 验证。

不要求独立编写七自由度解析 IK 求解器。

### 4.3 运动规划、碰撞检测与轨迹——完成

主要学习：

- Configuration Space（配置空间）
- OMPL 和 RRTConnect 双向树搜索
- 采样式碰撞检测
- Path 与 Trajectory 的区别
- 速度、加速度约束与时间参数化
- 规划计算时间与轨迹持续时间的区别
- 关节空间规划不保证末端直线运动

进行了重复规划、轨迹观察和速度缩放实验。

### 4.4 最小 C++ MoveIt2 规划程序——完成

创建 ROS2 C++ Package：

```text
src/openarm_moveit_learning/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── minimal_moveit_demo.cpp
│   └── scene_obstacle_demo.cpp
└── launch/
    └── learning_demo.launch.py
```

已经解决过：

- MoveIt2 CMake 编译与依赖链接问题
- `robot_description_semantic` 参数缺失
- ROS2 参数的节点作用域问题
- MoveGroupInterface 获取当前状态
- Executor 运行与 C++ 线程生命周期
- C++ 函数拆分、错误返回与资源清理

成功独立完成 C++ 关节目标规划。

**典型实验数据：**

- Joint1 目标增量：`+0.1 rad`
- 轨迹点数：约 30
- 轨迹持续时间：约 2.829 s
- 规划计算时间：约 0.019 s（单次样例）

### 4.5 轨迹执行与末端位姿目标——完成

已经实现：

**Joint Target：**

1. 获取当前关节状态。
2. 修改 Joint1 目标。
3. 调用 `plan()`。
4. 检查轨迹。
5. 调用 `execute()`。
6. 通过 `/joint_states` 验证执行后的关节位置。

其中一次 Joint1 执行后反馈：

```text
openarm_left_joint1 = 0.100065443 rad
```

**Pose Target：**

1. 读取当前末端 Pose。
2. 在 `world` 坐标系下令 Z 坐标增加 0.01 m。
3. 调用 `setPoseTarget()`。
4. 进行规划。
5. 使用 FK 检查规划终点。
6. 执行轨迹。
7. 读取执行后的末端位姿。
8. 计算位置与姿态误差。

一次完整验证记录：

| 指标 | 实验结果 |
|---|---:|
| 轨迹点数 | 35 |
| 规划时间 | 0.030 s |
| FK 规划终点位置误差 | 0.000099 m |
| 执行后位置误差 | 0.000099 m |
| 执行后姿态误差 | 0.000740 rad |
| 轨迹执行 | Success |
| 自定义精度检查 | PASSED |

这些都是 Mock Hardware 环境中的结果，不能代表真实机械臂的定位精度。

已通过代码重构实现多个运行模式：

```text
run_demo()
  ├── run_joint_demo()
  ├── run_pose_demo()
  ├── state 诊断模式
  └── ik_fail 诊断模式
```

通过 `learning_demo.launch.py` 的 `demo` 参数选择。

**注意：`pose` 和 `joint` 模式都可能执行增量运动，不要把它们当成纯状态检查命令。**

### 4.6 PlanningScene 与环境障碍物——完成

编写独立的：

```text
scene_obstacle_demo.cpp
```

通过 `PlanningSceneInterface` 实现：

- 添加 CollisionObject
- 查询碰撞物体 ID
- 查询尺寸与位姿
- RViz 障碍物显示
- 有障碍物 / 无障碍物运动规划对照
- 删除 CollisionObject
- 独立查询确认删除成功

使用的障碍物 ID：

```text
stage4_demo_box
```

实验立方体尺寸：

```text
0.1 × 0.1 × 0.1 m
```

通过 PlanningScene Service 验证 ADD 和 REMOVE 的效果。

有障碍物与无障碍物的单次规划记录：

| 实验 | 规划结果 | 规划耗时 |
|---|---|---:|
| 有障碍物 | 成功 | 0.0610 s |
| 无障碍物 | 成功 | 0.0217 s |

两次规划路径明显不同。但由于 RRTConnect 存在随机性，**不能仅根据这两次结果断定所有差异均由障碍物造成**。

已经理解三类对象的区别：

- RViz Visual Object：可视化对象
- MoveIt CollisionObject：碰撞检测几何体
- Physics Simulation Object：物理仿真对象

三者并不天然等价。

### 4.7 故障诊断与最终验收——完成

完成了三个可复现的故障案例。

**案例一：RobotState 读取失败**

正常情况下，`/joint_states` 由 `joint_state_broadcaster` 发布，观测平均频率接近 100 Hz。

通过 Topic Remapping，故意让学习节点订阅不存在的状态话题。

现象：

```text
Didn't receive robot state ...
Failed to fetch current robot state
RobotState unavailable after 2 seconds
```

根因：客户端没有收到有效关节状态。

恢复正确订阅后，`getCurrentState()` 重新成功。

**案例二：不可达末端目标**

故意设置 X 坐标约为 10 m 的目标。

客户端：

```text
MoveGroupInterface::plan() failed or timeout reached
error code: 99999
```

服务端：

```text
Unable to sample any valid states for goal tree
Invalid goal state
GOAL_STATE_INVALID
```

根因：目标远超机械臂可达工作空间，无法生成满足目标约束的有效状态。

理解了 `setPoseTarget()` 成功与 `plan()` 成功的区别，以及客户端通用错误码不一定提供完整故障原因。

**案例三：起点发生碰撞**

在 PlanningScene 中，将立方体放到机械臂当前连杆位置。

服务端：

```text
CheckStartStateCollision failed

2 contact(s) detected:
openarm_left_link3 - stage4_demo_box
openarm_left_link4 - stage4_demo_box

Aborting planning pipeline
```

根因：规划起始状态已经与障碍物发生碰撞。

此次故障在 PlanningRequestAdapter 阶段就被拦截，尚未进入 OMPL 实际路径搜索。

**工程结论：同样表现为 `plan()` 失败，根因却可能属于通信状态、目标可达性、碰撞约束或规划算法本身，必须逐层诊断。**

## 三、Stage 4 的核心收获

目前已具备以下能力：

1. 能解释 MoveIt2 从目标输入到轨迹执行的基本架构。
2. 能在 ROS2 Jazzy 下编写和调试 MoveGroupInterface C++ 程序。
3. 能独立完成关节空间目标和末端位姿目标规划。
4. 能执行规划轨迹，读取关节反馈与末端反馈。
5. 能利用 FK 验证规划终点并计算位置、姿态误差。
6. 能使用 PlanningScene 管理环境碰撞物体。
7. 能根据日志判断常见规划失败发生在哪一层。
8. 能区分规划器、轨迹控制器与 Fake Hardware 的职责。

至此，**Stage 4 核心验收全部完成**。

## 四、已知注意事项

- 当前使用 `mock_components/GenericSystem`，不是 MuJoCo 或 Gazebo 物理仿真。
- 之前曾出现 `minimal_moveit_demo` 的重复节点日志发布者警告，未影响实验。
- 曾观察到 `/move_group` 在 Ctrl+C 关闭期间出现 Segmentation Fault；它发生在资源清理阶段，不是规划碰撞实验的失败原因。若以后反复复现，再单独处理。
- `pose` 模式每次从当前位置 `Z += 0.01 m`；`joint` 模式每次从当前 Joint1 增加 `0.1 rad`，重复运行会累积运动。
- 重新启动 Fake Hardware 环境后，关节状态与 PlanningScene 可能重新初始化。不能假设上一次的姿态、障碍物仍然存在。
- 对规划器做严格性能比较，需要固定起止状态、记录随机性影响，并进行重复实验；Stage 4 不要求开发完整的 Benchmark 系统。

## 五、Stage 5 交接任务

**下一阶段：MoveIt Task Constructor（MTC）。**

Stage 4 解决的是：

> 给定机器人当前状态和运动目标，生成可执行的运动轨迹。

Stage 5 要进一步解决：

> 给定一个机器人操作任务，如何将任务分解为多个阶段，并组合、筛选各阶段的运动解？

预期学习方向：

- 理解 MTC 的 `Task`、`Stage`、`Solution` 等核心概念。
- 理解多阶段任务规划与普通 `MoveGroupInterface` 规划的区别。
- 搭建最小 MTC C++ 程序。
- 从简单的 MoveTo、Connect 等阶段开始。
- 逐步学习更完整的抓取和放置任务，包括相关的 IK、PlanningScene 与附着物体管理。
- 最终形成能够进一步对接机器人视觉和操作任务的基础框架。

**新的学习会话必须遵守以下要求：**

- 继续使用 Ubuntu 24.04 + ROS2 Jazzy。
- 继续使用现有 OpenArm 项目和 Fake Hardware。
- 优先复用已有 MoveIt2 配置，不无故重构整个工程。
- 采用小步教学：概念解释 → 最小代码 → 用户实际运行 → 分析日志 → 验收 → 下一步。
- 每次只推进一个清晰的小任务，不提前跳到后续小节。
- C++ 代码优先简洁、可理解、可手敲，必要时再加入工程级错误处理。
- ROS2 基础和配置知识够用即可，避免反复深挖已经完成的内容。
- 在未确认安全性之前，不执行真实硬件运动。
- 遇到错误先确定原因，不随意修改大量配置。
- 目标是具备独立使用 MTC 的工程能力，而非单纯记忆 API。

**下一会话的第一个任务：先审视 Stage 5 的整体学习大纲，然后从 Stage 5.1 的 MTC 核心架构开始，按照实际环境逐步实践。不要直接给出完整的复杂抓放系统。**

---

**交接状态：Stage 4 完成，准备启动 Stage 5。**