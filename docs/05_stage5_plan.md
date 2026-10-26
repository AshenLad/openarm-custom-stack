# OpenArm Stage 5：MoveIt Task Constructor 与程序化抓放

**版本：V1.1 · 2026-10-26**

**当前进度：Stage 0～4 已完成，进入 Stage 5。**

**本阶段目标：独立编写简化版 MTC C++ 程序，在 Fake Hardware 中完成单臂 Pick & Place，并实现左右臂分别抓取目标的基本流程。**

计划依据：[总体路线图](ROADMAP.md)、[Stage 4 计划](04_stage4_plan.md)、[Stage 4 完结总结](04_stage_summary.md)。本文是后续教学与实验安排，文中的待开发文件和验收项尚未完成。

本版修订：增加贯穿抓放流程的状态转移核对表；明确双臂顺序任务中必须在左臂执行后重新读取状态并重建右臂任务；将时间估算明确为弹性预算而非硬性期限。

---

## 一、阶段定位与系统全景

Stage 4 已解决“给定一个目标，如何规划、执行并检查运动结果”。Stage 5 要解决“抓取任务包含多个动作和场景变化，如何组合出一条完整可行的任务解”。

可以把 Stage 4 的轨迹看成一道工序的施工方案；MTC 则负责组织整套工序，检查前后状态是否衔接，并在候选抓取姿态、IK 和运动路径之间寻找可行组合。

```text
[固定物体位姿 + 放置目标 + 当前机器人状态]
                         |
                         v
 [★ Stage 5：学员手写 MTC Task]
   CurrentState -> 打开夹爪 -> 接近 -> 抓取 -> 抬升
                  -> 搬运 -> 放置 -> 撤离 -> 返回
                         |
        Stage / Container / Solution
                         |
             运动规划 + PlanningScene 状态变化
                         |
                         v
 [move_group：MTC 任务执行能力]
                         |
             机械臂 / 夹爪控制器 Action
                         |
                         v
 [ros2_control] -> [mock_components/GenericSystem]
                         |
                  /joint_states
                         |
                  [结果观测与验证]
后续接口：Stage 7 外部 Pose -> Stage 8 GDRNPP 位姿估计
后续物理：Stage 6 Gazebo 接触、重力、执行误差
```

上图描述任务行为顺序。MTC 内部求解可以从生成器向前、向后传播，再由 Connect 衔接；不能把它理解成按源码顺序逐个调用 `plan()` 的脚本。

参考：[MTC 基本概念](https://moveit.github.io/moveit_task_constructor/basics.html)。

本阶段沿用 **CachyOS + Distrobox + Ubuntu 24.04 + ROS 2 Jazzy + C++17**。所有运动实验限定在已经核实的 Fake Hardware 环境。

## 二、从 Stage 4 继承什么

已经完成的能力直接复用，遇到具体问题再补充：

- `MoveGroupInterface` 关节目标、末端位姿目标、规划与执行。
- RobotState、FK / IK、`world` 坐标下的位姿和误差检查。
- PlanningScene 添加、查询、删除碰撞物体。
- Executor 与后台线程、节点参数作用域、CMake 和安装规则。
- 状态读取失败、不可达目标和起点碰撞的分层诊断。

新增重点是 **任务解的组合、Stage 间状态传递、抓取坐标关系、附着物体生命周期和多阶段失败定位**。

### 2.1 已确认的项目基础

| 项目 | 当前依据 | Stage 5 用法 |
|---|---|---|
| Stage 4 学习包 | `src/openarm_moveit_learning/` | 保留已完成实验，复用构建和参数加载经验 |
| 已有 MTC 包 | `src/openarm_mtc_pick/` | 作为可运行基线和按需查阅的工程参考 |
| Fake 演示入口 | `fake_pick_place.launch.py` | 先只规划，确认环境后执行 |
| Fake 演示配置 | `config/fake_pick_place.yaml` | 当前使用右臂，物体与支撑面由固定场景提供 |
| Stage 4 主臂 | `left_arm` | 手写最小 MTC 从熟悉的左臂起步 |
| 规划参考系 | `world` | 初期固定物体与放置目标统一在此表达 |
| 机器人模型 | `openarm_v20` / `openarm_v2.0` 配置目录 | 继续复用现有模型与 MoveIt 配置 |

这些是文档和源码确认的事实；已有演示本轮没有重新运行，运行状态在 5.1～5.2 验证。

### 2.2 开始前需要核对的差异

1. **左右臂配置分开核对。** 当前 SRDF 定义了 `left_arm/right_arm`、`left_gripper/right_gripper`、`left_ee/right_ee`。末端 Link 分别为 `openarm_left_ee_base_link` 和 `openarm_right_ee_base_link`，EEF 名称、Planning Group 与 Link 不是同一种标识。

2. **夹爪没有现成的 `open/closed` 命名状态。** 已检查的 v2.0 SRDF 只有机械臂 `home/hands_up` 命名状态。学习程序先使用经限位核对的夹爪关节目标，不能直接照搬 Panda 教程的 `setGoal("open")`。

3. **夹爪 Action 类型必须运行时确认。** 当前 MoveIt 控制器 YAML 对左右夹爪的类型配置不同；还需结合 Launch 实际加载的 ros2_control 配置和 `ros2 action list -t` 判断，不能只凭某个 YAML 判定接口可用。

4. **TCP 与末端 Link 的固定变换必须从模型确认。** 现有右臂演示配置了 `openarm_right_grasp_frame`；手写程序使用的 TCP 是否存在、如何从末端 Link 得到，都要核实。

---

## 三、最终验收标准

完成 Stage 5 必须具备以下结果：

1. 跑通已有 Fake Pick & Place，分别保留规划与执行证据。
2. 独立编写简化 MTC C++ 程序，能够解释每个 Stage 的输入、输出和作用。
3. 程序完成打开夹爪、接近、抓取、Attach、抬升、搬运、放置、Detach、撤离的闭环。
4. 能修改抓取姿态、接近距离、物体位置，并解释成功或失败发生在哪个阶段。
5. 同一套任务构建逻辑支持左右臂分别操作各自目标，完成顺序执行实验。
6. 至少保留三个可复现、可解释的失败案例。
7. 从干净初始场景按记录步骤重复运行，结果能通过关节反馈与场景查询验证。
8. 在 5.4～5.6 维护状态转移记录，至少记录机器人状态、世界物体与附着物体归属、必要碰撞例外及其验证证据。

**完成含义：任务规划、控制器执行以及 PlanningScene 状态变化在 Mock 环境中闭环。物体附着成功不能证明真实夹持、摩擦或物理抓取成功。**

## 四、分阶段学习与实践

每节可拆成多次小实验。实际教学每次只推进一个可验证微步骤，收到终端输出并完成解释后再继续；下面的整体计划不要求一次性实现。

### 5.1 MTC 架构与实际环境

**目标：明确 MTC 增加在哪一层，确认模型、规划与执行接口可用。**

学习内容：

- `Task`、`Stage`、`Container`、`Solution` 的职责。
- Generator、Propagator、Connector 的状态流向。
- `init()`、`plan()`、解的可视化、`execute()` 的边界。
- MTC 客户端加载模型和规划器，与 `move_group` 执行任务的关系。

实践按顺序拆开：

1. 只读检查现有 Fake Launch、MTC 依赖、机器人模型和控制器配置。
2. 确认容器内实际使用的包前缀与源码版本，排除旧 overlay。
3. 启动已确认的 Fake 环境，查看节点、状态话题、控制器和 Action。
4. 核对 `/execute_task_solution` 及其类型；现有 `demo.launch.py` 配置了 `move_group/ExecuteTaskSolutionCapability`，仍需确认服务端运行正常。
5. 按实际启动方式运行已有的 `mtc_environment_check`，观察仅含 `CurrentState` 的任务能否取得场景并产生解。

**交付：** 一张实际系统拓扑图，以及左右臂 Group、EEF、Link、TCP、夹爪关节和 Action 对照表。

**验收：** 能说明“模型加载成功”“取得当前场景”“任务规划成功”“任务执行成功”分别证明了什么。

### 5.2 跑通已有 Fake Pick & Place

**目标：先看到完整任务结果，建立后续手写程序的参照。**

使用已存在的右臂演示；先确认没有另一套机器人启动栈在同一 ROS domain 内运行，并核实实际硬件插件为 `mock_components/GenericSystem`。

已核对的入口：

```bash
# 容器内，已加载 Jazzy 和本工作空间 overlay；先只规划
ros2 launch openarm_mtc_pick fake_pick_place.launch.py execute_fake:=false

# 前一次进程已退出，完成规划检查与 Mock 环境确认后再运行
ros2 launch openarm_mtc_pick fake_pick_place.launch.py execute_fake:=true
```

演示 Launch 自带 Fake MoveIt 启动流程；不要再叠加一套 `demo.launch.py`。默认关闭 RViz，后续按需连接 RViz 查看任务解，终端日志和状态反馈也必须保留。

记录四类证据：任务树和完整解、执行返回结果、关节反馈、物体最终场景状态。执行完成以最终结果为准，不能只看 Goal 被接受或动画播放。

**验收：** 能将演示动作对应到抓取、附着、搬运、释放等阶段，并指出哪些行为只是 Mock 场景状态变化。

**范围控制：** 现有程序还包含视觉检查和较多工程逻辑。本节只追踪任务主线，不逐行讲解整个 `mtc_pregrasp.cpp`。

### 5.3 独立编写最小 MTC C++ 程序

**目标：从熟悉的左臂单次运动，建立最小 Task 的编写、构建与执行闭环。**

拟新增学习包 `openarm_mtc_learning`，复用原有 OpenArm 模型和 MoveIt 配置。学员独立敲写 C++；导师提供接口说明、局部骨架和代码审查。

依次实现两个微实验：

```text
实验 A：CurrentState -> 解的输出与检查
实验 B：CurrentState -> MoveTo（已核对的左臂关节目标）
```

学习 `Task` 的创建、机器人模型加载、Stage 所有权移交、初始化错误输出、解集合检查和执行结果检查。先只规划，确认轨迹后再在 Mock 中执行。

Launch 必须为学习节点提供它实际需要的模型、SRDF、IK、关节限制与规划流水线参数。Stage 4 客户端能运行，不代表 MTC 客户端已经拿到了本地规划所需的全部参数。

**验收：** 程序可编译、可从安装目录启动；有解才允许执行，执行后从 `/joint_states` 验证目标。异常退出时后台 spin 线程能正常清理。

### 5.4 抓取姿态、ComputeIK 与 Connect

**目标：从“物体位姿”得到“机械臂能到达的抓取状态”。**

在固定场景中加入一个简单物体，先使用少量抓取候选：

```text
[CurrentState -> 打开夹爪] --Connect-- [抓取目标侧状态]
                                           ^
                              ComputeIK(GenerateGraspPose)
```

学习重点：

- `GenerateGraspPose` 生成候选目标，`ComputeIK` 将候选转换为关节解。
- `Connect` 衔接两侧已得到的兼容状态；有 IK 解仍不保证连接路径存在。
- `group`、`eef`、`ik_frame`、`target_pose` 的来源和含义。
- 父容器属性与接口属性的传递；只在当前任务需要时学习 `PARENT/INTERFACE`。
- 被监视 Stage 提供哪个场景状态，以及它的生命周期。

坐标关系按以下记号统一：`T_A_B` 表示 B 坐标系在 A 中的位姿。

```text
T_world_tcp = T_world_object * T_object_tcp
T_world_ee  = T_world_tcp * inverse(T_ee_tcp)
```

先说明几何目标，再核对 `setIKFrame()` 的具体语义，避免代码重复施加 TCP 偏移。位置用 m、角度用 rad；ROS 四元数顺序为 `(x, y, z, w)`，Eigen 构造顺序为 `(w, x, y, z)`，有效四元数使用前归一化。

**验收：** 至少得到一条连接到抓取状态的可行解；能区分“没有生成有效候选”“IK 无解”和“Connect 失败”。本节不要求已经完成夹持。

### 5.5 接近、夹紧、Attach 与抬升

**目标：把到达抓取位置扩展成一个有正确场景变化的 Pick 子任务。**

使用 `SerialContainer` 组织抓取阶段：

```text
接近 MoveRelative -> 抓取状态 -> 允许必要接触
       -> 关闭夹爪 MoveTo -> Attach -> 抬升 MoveRelative
```

以上是行为顺序；实现时要结合 5.4 的生成器和传播方向安排 Stage 接口。

每次只增加一个机制：

1. 接近与抬升：用 `MoveRelative` 和笛卡尔规划器，明确方向向量所在坐标系和距离范围。
2. 夹爪开合：使用已核对的关节值、限位和实际控制器接口。
3. 接触与附着：通过 `ModifyPlanningScene` 设置必要碰撞例外并 Attach 物体。
4. 验证抬升时附着物体随机器人参与后续碰撞检查。

| 状态 | 世界物体 | Attached Object | 应验证的含义 |
|---|---|---|---|
| 抓取前 | 目标存在 | 无该目标 | 物体位姿固定在环境中 |
| Attach 后 | 目标从世界物体集合移出 | 附着到指定 Link | 物体相对该 Link 的位姿保持固定 |
| 抬升后 | 无独立的该目标 | 继续附着 | 后续规划考虑持物几何 |

只放开物体与指定接触 Link 等必要碰撞对。若支撑面接触需要例外，明确允许和恢复的阶段，不得关闭整机碰撞检查。

**验收：** 可以规划并在 Mock 中执行到抬升结束，通过场景查询确认附着对象 ID 和 Link。规划分支中的场景变化与执行后服务端场景必须分别验证。

### 5.6 放置、Detach 与完整 Pick & Place

**目标：形成单臂完整抓放闭环。**

```text
持物状态 -> Connect -> 放置接近 -> 放置状态
        -> 打开夹爪 -> Detach -> 撤离 -> 返回
```

学习 `GeneratePlacePose` 与 `ComputeIK` 的组合，确保放置候选继承正确的持物状态。目标物体的放置位姿与末端目标位姿要通过附着关系转换，不能直接把物体中心当作 TCP 目标。

实践：

1. 先设置一个固定可达的放置目标，完成持物搬运和下降规划。
2. 添加打开夹爪、Detach、撤离与必要碰撞规则恢复。
3. 返回记录的起始关节状态或已验证的安全关节目标。
4. 规划完整任务，选择一条完整解，执行该解并检查终态。
5. 从重置后的相同场景重复运行，检查物体残留、附着残留和初始状态。

**验收：** 完整任务执行成功；目标已 Detach 并出现在世界场景的预期放置位姿，机器人最终关节反馈符合目标。返回动作结束后的 TCP 不能用来衡量先前放置点误差。

#### 贯穿 5.4～5.6 的状态转移核对表

每次增加一个 Stage，都要更新这张表。表中的内容是预期状态，不是未经验证的事实；具体查询命令和接口以当前 Jazzy/MTC 环境实际提供的能力为准。

| 阶段 | 机器人预期状态 | 物体预期状态 | 必须保留的验证证据 |
|---|---|---|---|
| 初始状态 | 从当前关节反馈建立起始状态 | 目标在世界物体集合中，不在附着物体集合中 | 初始关节状态、物体 ID 与位姿、场景快照 |
| 接近与抓取目标 | 末端到达候选抓取状态附近 | Attach 前仍属于世界物体 | 任务解对应的抓取目标、相关 Stage 结果 |
| 关闭夹爪并 Attach | 夹爪执行已核对的关节目标 | 目标从世界物体集合移出，并附着到预期 Link | 执行结果；世界物体与附着物体查询；附着 Link 和相对位姿 |
| 抬升与搬运 | 机器人沿规划路径持物运动 | 附着关系持续存在，持物几何参与后续碰撞检查 | 完整解中的阶段结果、执行后场景状态 |
| 放置接近与下降 | 末端到达放置目标对应的状态 | Detach 前仍附着于机器人 | 放置目标推导、任务解与执行结果 |
| 打开夹爪并 Detach | 夹爪执行打开目标，机器人与物体分离 | 目标不再附着，并回到世界物体集合中的预期放置位姿 | 附着集合与世界集合查询、最终物体位姿 |
| 撤离与返回 | 机器人到达记录的安全目标或起始关节目标 | 放置物体留在世界场景中 | 最终关节反馈、场景终态、执行结果 |

**重要区分：** MTC 求解过程中各分支的场景状态，与任务执行结束后服务端的实际 PlanningScene 状态，必须分别核对。夹爪关闭或 Attach 成功本身不证明发生了真实物理夹持。

### 5.7 单变量实验与任务失败分析

**目标：理解任务为什么有解、为什么失败，完成 Roadmap 要求的参数修改实验。**

| 实验 | 只修改的输入 | 主要观察 |
|---|---|---|
| 抓取姿态 | 抓取方向或候选角度 | 候选数量、IK、连接路径和关节限位 |
| 接近距离 | 接近距离范围 | 直线路径、碰撞和可达范围 |
| 物体位置 | `world` 中的物体位置 | 抓取与放置两段是否仍有完整解 |

每组先保留可成功的基线，再逐步修改；每次重置初始状态和场景。记录失败样本，不通过同时修改多个参数掩盖原因。

至少完成三类故障：

- 初始化或属性错误：例如错误的 Group、缺失的必要属性。
- 几何规划失败：例如不可达抓取目标、接近路径碰撞。
- 执行链路失败：在 Fake 中使指定控制器不可用，区分完整任务有解与执行失败；实验后恢复控制器。

**验收：** 能根据初始化异常、Stage 解/失败信息、碰撞对、Action 结果逐层定位。任务树里某个 Stage 没有解可能是上游未提供输入，不能一律认定它就是根因。

### 5.8 左右臂分别抓取目标

**目标：完成 Roadmap 的双臂基本流程，形成可复用任务构建逻辑。**

将已经跑通的单臂程序提取为接收机械臂配置和任务输入的构建函数，参数至少覆盖 Group、夹爪、EEF、末端/TCP、物体 ID、抓放目标和开合目标。

按顺序验证：

1. 左臂抓放自己的目标，右臂保持在已知安全姿态。
2. 右臂抓放自己的目标，左臂保持在已知安全姿态。
3. 在同一场景放置两个不同 ID 的目标。左臂任务执行结束并确认最终执行结果后，重新读取 `/joint_states` 与 PlanningScene；基于更新后的机器人状态和场景，重新构建右臂 Task，再规划并执行右臂任务。不得复用左臂执行前生成的旧 Task 或旧 Solution。

两臂始终保留在完整机器人碰撞模型中。先完成的放置物体必须继续参与另一臂的碰撞检查。一次双臂顺序试验中，不得在左右臂任务之间重置场景；只有开始下一次独立复现实验时，才按记录恢复干净初始状态。

**验收：** 同一份核心任务逻辑支持两臂，完成顺序抓放并分别记录执行与场景结果。双臂同时运动、交接、相对位姿对齐属于 Stage 9。

### 5.9 阶段复盘与交付

从干净启动状态按文档复现：已有演示、手写单臂任务、参数变化实验、失败案例、双臂顺序任务。

固定场景至少重复三次，逐次记录成功或失败及原因。每次独立重复开始前，都从记录的干净初始状态恢复机器人与场景；但在单次双臂顺序试验内部，左臂完成后必须保留其放置结果供右臂规划使用。这只是基础可重复性检查，不据此宣称系统具有工业可靠性或真实抓取成功率。

最终独立解释：

```text
Stage 3：Trajectory -> Controller -> Hardware
Stage 4：Target -> Motion Planning -> Trajectory
Stage 5：Task -> Stages / Scene States -> Complete Solution -> Execution
```

---

## 五、代码、构建与实验记录

### 拟新增的学习交付物

下面是后续逐步创建的结构，本次制定计划不生成完整抓放实现：

```text
src/openarm_mtc_learning/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── minimal_mtc_demo.cpp
│   └── pick_place_demo.cpp
├── launch/
│   └── learning_mtc.launch.py
└── config/
    └── learning_task.yaml
docs/
├── 05_stage5_plan.md
├── 05_mtc_basics.md       # 学员实验记录，逐步填写
└── 05_stage_summary.md   # 完成验收后再写
```

初期只创建最小实验需要的文件。抓放程序成熟后再提取共用函数，避免过早设计框架。

### 构建闭环

每次新增接口或依赖，同步核对 `package.xml` 和 `CMakeLists.txt`。核心依赖包括 `rclcpp`、`moveit_task_constructor_core`；场景消息、几何消息和其他库按实际使用添加。

CMake 必须覆盖 C++17、`find_package`、目标链接、`ament_target_dependencies`、`install(TARGETS ... DESTINATION lib/${PROJECT_NAME})`，以及 Launch/配置目录安装。Launch 使用的 Python 包也应在清单中声明运行依赖。

后续每个代码实验都提供完整链路：

```text
学员编写/修改
  -> 核对 CMakeLists.txt + package.xml
  -> colcon build --packages-select openarm_mtc_learning
  -> source install/setup.bash
  -> 启动已确认的 Fake 基础环境
  -> 运行学习程序
  -> CLI / 任务解 / PlanningScene 验证
```

依赖包需事先已构建并加载；若尚未满足，先定位缺失依赖再补齐。具体 Launch 参数随实现定义，不把本文拟定名称当成已经存在的命令。

每次实验明确容器、工作空间、终端用途与 overlay。只读查询无需重新编译；文档修改也无需启动机器人。

### 记录字段

| 字段 | 含义 |
|---|---|
| `arm_group / object_id` | 本次使用的机械臂与目标 |
| `initial_state / scene` | 起始关节和碰撞场景，可复现实验的前提 |
| `object_pose / place_pose` | 输入位姿，含坐标系与单位 |
| `grasp_parameters / approach_distance` | 抓取设置和接近距离 |
| `planning_success / solution_count` | 是否存在完整任务解及其数量 |
| `planning_time / trajectory_duration` | 规划耗时与运动轨迹时长，分开记录 |
| `failure_stage / evidence` | 失败环节及日志、碰撞或接口依据 |
| `stage_state_transition` | 每个关键 Stage 前后的机器人状态、世界物体 ID、附着物体 ID 与必要碰撞例外 |
| `execution_result` | 最终执行结果，不使用 Goal 接受状态代替 |
| `final_joint_error / final_object_state` | 关节误差与目标所在场景集合、位姿 |

先用 Markdown 表格或简单 CSV，不开发复杂 Benchmark 系统。

## 六、避坑与面试赋能

### 工程排雷

1. **所有权：** `task.add(std::move(stage))` 后原 `unique_ptr` 为空，禁止继续通过它调用方法。监视 Stage 所用的非拥有指针必须对应仍由 Task/Container 持有的对象，任务重建后不能复用旧指针。

2. **版本：** 以容器中实际安装或构建的 Jazzy 依赖为准。本仓库 MTC 使用 `task.h`、`stages/compute_ik.h` 等真实存在的 ROS 2 头文件；不能仅为统一扩展名改成不存在的 `.hpp`。ROS 2 消息及 MoveIt 接口按实际提供的 `.hpp` 使用。

3. **场景与控制：** 核对物体 ID、EEF、Link、TCP 和 Action 类型。恢复碰撞规则时要结合撤离过程，避免尚在接触就引入碰撞，也避免任务结束后遗留不必要例外。

4. **C++ 生命周期：** 消息智能指针使用 `->`；回调不捕获即将销毁的局部引用；成功和异常路径都要处理 Executor 停止与线程回收。

### 面试快问快答

**MTC 与连续调用 MoveGroupInterface 有什么区别？**

MTC 将多阶段任务表达为带机器人和场景状态的子问题，并组合候选解。某个抓取姿态有 IK 解，还需要与接近、搬运、放置等阶段共同形成完整可行解。

**GenerateGraspPose、ComputeIK、Connect 各做什么？**

三者分别负责抓取目标候选、目标对应的关节解，以及两侧状态之间的运动连接。三层都可能失败，所以“目标已生成”不能代表“机器人能完成抓取”。

**Attach 是否意味着真的抓住了物体？**

Attach 改变 PlanningScene 中物体与机器人 Link 的关系，使后续规划按持物几何检查碰撞。它不验证接触力、摩擦和滑落，Fake Hardware 也无法提供这些物理证据。

**为什么规划成功仍可能执行失败？**

规划证明在模型和场景条件下存在任务解；执行还依赖控制器、Action 接口、当前状态以及轨迹约束。应结合最终执行结果、关节反馈和场景终态判断，不能以 Goal 接受为成功依据。

## 七、范围、时间与止损

建议按约 **3～5 周、累计 18～30 小时**安排，只作为初始预算区间，不是硬性期限或质量指标。若环境、API 或控制器问题导致某一环节超时，优先回退到最近一次成功的最小任务，保留日志并定位根因；不要为了赶进度同时引入多个新机制。

| 内容 | 建议投入 |
|---|---:|
| 5.1～5.2 架构与已有演示 | 3～5 小时 |
| 5.3～5.4 最小程序与抓取候选 | 4～6 小时 |
| 5.5～5.6 完整单臂抓放 | 6～10 小时 |
| 5.7～5.9 变化实验、双臂与复盘 | 5～9 小时 |

必须深入：Stage 数据流、完整 Solution、TCP、Attach/Detach、碰撞例外和失败分析。Alternatives、Fallbacks、自定义代价等按需了解；自定义 Stage 与复杂任务框架作为选修。

后续阶段处理：Gazebo 物理抓取、RGB-D 与 CameraInfo、外部 Pose 接口、GDRNPP、双臂协调对齐、MuJoCo 与 Sim-to-Sim。已有仓库包含相关实现不代表本阶段需要全部讲解。

卡住时先把任务缩回最近一次成功的最小结构，固定场景、减少候选，再定位配置、几何规划或执行层。保留已有程序作为对照，按根因做单变量修改。

## 八、下一会话教学协议

1. 先阅读本计划与 Stage 4 总结，承认 Stage 4 核心验收已完成。
2. 从 **5.1：MTC 架构与实际环境** 开始，每次只安排一个微任务。
3. 先画 ASCII 拓扑，说明当前输入、输出、坐标系与成功标准。
4. 读取真实文件或终端输出后再给修改方案，未知名称先探针确认。
5. 学员独立手写 C++；导师给必要接口、局部示例、构建要求和代码审查。
6. 每步覆盖编写、依赖、编译、加载环境、运行与观测，明确多终端职责。
7. 只在已确认的 Mock 环境执行；规划、执行与物理抓取的结论分别表述。
8. 报错先定位断点，用一个最小排查动作推进，反馈验收后再进入下一步。

**第一条微任务：核对已有 MTC Fake 入口及环境检查程序的实际启动配置，画出它们与 `move_group`、控制器和 `/joint_states` 的关系。此步只读，不生成完整抓放程序。**

参考资料以本地 `third_party/moveit_task_constructor` 和容器实际头文件为 API 依据；[MTC 基本概念](https://moveit.github.io/moveit_task_constructor/basics.html) 与 [MoveIt 官方抓放教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) 用于理解结构，后者为 Rolling 文档，示例须核对后再用于 Jazzy/OpenArm。