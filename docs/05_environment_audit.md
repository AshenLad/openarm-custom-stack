# Stage 5 环境审计、修复与运行验证报告

**审计基线提交：`2b10da2`。范围：现有环境、已有演示及必要配置修复。**

执行时间以 [system.txt](../log/stage5_audit_f611c7zb/system.txt) 的 Asia/Shanghai 时间戳为准。本文中的运行结论来自本轮实际测试；日志目录 `log/stage5_audit_f611c7zb/` 被仓库现有规则忽略，交接或迁移机器时应另外保存。

## A. 总结结论

**当前环境可以开始 Stage 5。已有右臂 Fake Pick & Place 的规划与完整执行通过；进程退出清理仍存在已复现的非阻断崩溃。**

- 初次只规划生成 **5 条完整解**，关节反馈未发生运动；最后一次重新启动验证也生成 5 条完整解。
- 第一次执行尝试因 `/joint_states` 尚未就绪而失败，未发生运动。调整 Fake 初始状态等待时间后，完整执行返回 `SUCCEEDED`，程序报告 `FULL TEST COMPLETE - robot returned to initial joints`。
- 独立场景采样确认 `blue_cube` 经历“世界物体 → 附着于 `openarm_right_ee_base_link` → 世界物体”的变化；最终没有残留该附着对象。
- 修复了左夹爪 MoveIt Action 配置与实际控制器不匹配的问题。左右夹爪通过 MoveIt 完成小幅开合与复位，均取得最终成功结果和一致的关节反馈。
- 左右臂最小 `CurrentState` 检查、Stage 4 左臂关节规划执行、PlanningScene 增删和左右 TCP TF 查询通过。
- 两个相关包定向构建通过；两个已有 MTC CTest 目标通过，`colcon test-result` 汇总 **26 tests，0 errors，0 failures**。
- 本轮只启动 Mock 硬件，未启动真实硬件控制。最后检查 domain 86 无残留节点。

**当前没有发现阻断 Stage 5 基础学习的未解决问题。** 但不能把上述结果表述为“所有进程均干净退出”或“真实物理抓取成功”。左臂完整抓放和双臂顺序抓放尚未实现或验证，仍是学习任务。

关键证据：[完整执行日志](../log/stage5_audit_f611c7zb/retry/fake_execute.log)、[执行期间场景采样](../log/stage5_audit_f611c7zb/retry/fake_execute_scenes.json)、[接口回归结果](../log/stage5_audit_f611c7zb/regression/interfaces_result.json)、[最终只规划](../log/stage5_audit_f611c7zb/final_plan/fake_plan.log)、[清理结果](../log/stage5_audit_f611c7zb/cleanup.json)。

## B. 环境基线与证据

### 宿主、容器、软件与工作空间

| 项目 | 实测结果 | 状态 / 证据 |
|---|---|---|
| 宿主 | CachyOS；可读取 `/run/host/etc/os-release` | 已验证；[系统记录](../log/stage5_audit_f611c7zb/system.txt) |
| 容器 | 名称 `ros-jazzy`；Podman 6.1.4，rootless；存在 `distrobox-host-exec` | 已验证容器标识与 Distrobox 集成工具；未调用宿主容器管理命令 |
| 容器系统 | Ubuntu 24.04.4 LTS | 已验证；`/etc/os-release` |
| ROS 2 | Jazzy；`ros2cli` 0.32.12；安装于 `/opt/ros/jazzy` | 已验证；[包版本和前缀](../log/stage5_audit_f611c7zb/baseline.txt) |
| MoveIt 2 | 2.12.4；实际加载 `/opt/ros/jazzy/lib/libmoveit_robot_model.so.2.12.4` | 已验证；[动态链接与 Debian 包版本](../log/stage5_audit_f611c7zb/linkage.txt) |
| MTC | 包版本 0.1.5；源码提交 `56c6a2a40ebff8a4d1644d9e6fac819c3cd3542f` | 已验证源码、安装头文件、CMake 包及实际动态链接路径 |
| 编译器 | GCC/G++ 13.3.0；MTC 使用 `-std=gnu++17` | 已验证；构建参数及编译器输出 |
| 学习包 C++ 标准 | 当前 G++ 默认 `__cplusplus=201703L`，即 C++17 | 已验证；学习包目前依赖编译器默认值，未为风格统一修改其 CMake |
| 构建工具 | CMake 3.28.3；Python 3.12.3 | 已验证；[系统记录](../log/stage5_audit_f611c7zb/system.txt) |
| 工作空间 | `/home/ashenlad/code/openarm-custom-stack`；`colcon list` 识别 17 个包 | 已验证；包来自 `src/` 与 `third_party/` |
| 学习包前缀 | `<workspace>/install/openarm_moveit_learning` | 已验证安装可执行程序、Launch 与实际运行 |
| MTC 演示前缀 | `<workspace>/install/openarm_mtc_pick` | 已验证安装程序、Launch、配置与实际运行 |
| MTC core 前缀 | `<workspace>/install/moveit_task_constructor_core` | 已验证；不是另一份系统 MTC 覆盖当前包 |
| Shell 环境 | 初始非交互 zsh 中 ROS/AMENT/CMAKE/COLCON 变量未设置；显式 source 后加载正常 | 已验证；不能把 PATH 中存在 `ros2` 等同于已加载 ROS 环境 |
| Overlay | source 后 AMENT/CMAKE 路径来自当前工作空间与 Jazzy；`COLCON_PREFIX_PATH=<workspace>/install` | 已验证；未发现另一套工作空间覆盖 |
| 路径别名 | 部分旧缓存和符号链接使用 `/run/host/home/...`，与 `/home/...` 实测为同一 inode | 已验证；不是第二份源码，但会影响沙箱写入权限 |
| ROS domain | 初始未设置，默认 0；本机只读查询无节点。测试使用 86、`LOCALHOST`，每次启动前检查空图 | 已验证本机发现范围；没有扫描整个局域网或其他 domain |

初始根仓库 `git status --short` 为空。`third_party/openarm_ros2` 和 `third_party/openarm_description` 有已有集成修改；它们在审计前已保存差异快照。`openarm_description` 和 MTC 的原有差异完全保留，OpenArm ROS 2 上游工作副本仅新增左夹爪映射修正。见 [保留检查](../log/stage5_audit_f611c7zb/preservation.txt) 与 [相对初始状态的上游差异](../log/stage5_audit_f611c7zb/upstream_change_delta.diff)。

### 模型、左右臂与 TCP

| 项目 | 左臂 | 右臂 | 证据 / 状态 |
|---|---|---|---|
| Robot / 根坐标系 | `openarm_v20` / `world` | 同左 | 运行时 URDF、模型加载日志已验证 |
| Arm Group | `left_arm`，7 个关节 | `right_arm`，7 个关节 | SRDF + 两侧 CurrentState 检查通过 |
| Gripper Group | `left_gripper` | `right_gripper` | SRDF 与模型加载已验证 |
| EEF 名称 | `left_ee` | `right_ee` | SRDF 已核实；右臂 MTC 抓放实际使用 |
| 末端 Link | `openarm_left_ee_base_link` | `openarm_right_ee_base_link` | 运行时模型与 TF 已验证 |
| TCP | `openarm_left_grasp_frame` | `openarm_right_grasp_frame` | 两侧实际 TF 查询通过 |
| 夹爪主动关节 | `openarm_left_finger_joint1` | `openarm_right_finger_joint1` | URDF、控制器映射、反馈已验证 |
| URDF 夹爪限位 | `[0, 0.7854] rad` | `[-0.7854, 0] rad` | 展开 URDF 已验证；左右符号不同 |
| 本轮开合回归 | `0 → +0.2 → 0 rad` | `0 → -0.2 → 0 rad` | MoveIt 执行最终状态均为成功；目标反馈误差为 0 |
| 已有右臂演示目标 | 不直接复用右臂负角度配置 | 打开 `-0.35 rad`；本次动态闭合约 `-0.043 rad`；释放撤离后收回 0 | 右臂实际执行日志；左臂抓取目标仍需后续任务验证 |
| IK | KDL；配置超时 0.005 s | KDL；配置超时 0.020 s | 配置及运行时求解器参数已验证 |

`T_ee_tcp` 两侧实测相同：平移约 `(-0.00143, 0, -0.138) m`，四元数 `(x,y,z,w) ≈ (0, 0.70710678, 0, 0.70710678)`。这是末端 Link 到 TCP 的固定关系，不是 TCP 在 `world` 中的绝对位姿。

证据：[静态模型审计](../log/stage5_audit_f611c7zb/static_audit.json)、[实际运行模型](../log/stage5_audit_f611c7zb/retry/fake_execute_runtime.urdf)、[左右 TCP TF](../log/stage5_audit_f611c7zb/regression/tcp_tf.json)、[左臂 CurrentState](../log/stage5_audit_f611c7zb/regression/left_environment.log)、[右臂 CurrentState](../log/stage5_audit_f611c7zb/environment_check.log)。

v2.0 SRDF 中存在两臂 `home/hands_up`，未定义夹爪 `open/closed` 命名状态。最小学习程序应使用已核实的关节目标，不应照搬其他机器人命名状态。

### 控制器、Action 与参数

| 实际名称 | 运行类型 / 接口 | 状态与执行证据 |
|---|---|---|
| `openarm_left_hardware_interface` / `openarm_right_hardware_interface` | `mock_components/GenericSystem` | 两侧 active；由硬件组件查询和运行时 URDF 交叉验证 |
| `left_joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController`；`/left_joint_trajectory_controller/follow_joint_trajectory` | active；Stage 4 C++ 左臂规划执行成功 |
| `right_joint_trajectory_controller` | 同上；`/right_joint_trajectory_controller/follow_joint_trajectory` | active；MTC 完整抓放执行成功 |
| `left_gripper_controller` | 同上；`/left_gripper_controller/follow_joint_trajectory` | active；修复后经 MoveIt 开合与复位通过 |
| `right_gripper_controller` | 同上；`/right_gripper_controller/follow_joint_trajectory` | active；开合回归与完整抓放通过 |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active；实际发布 `/joint_states`，约 100 Hz |
| `/execute_task_solution` | **Action**：`moveit_task_constructor_msgs/action/ExecuteTaskSolution` | 已发现服务端，且完整 MTC 解实际执行成功 |
| `/execute_trajectory` | Action：`moveit_msgs/action/ExecuteTrajectory` | 左右夹爪与左臂执行链路验证通过 |
| `/move_action` | Action：`moveit_msgs/action/MoveGroup` | Stage 4 左臂程序实际使用 |
| `/get_planning_scene` / `/apply_planning_scene` | MoveIt 场景查询 / 应用 Service | 世界物体添加、查询、删除与执行期间采样通过 |

四个控制器的轨迹 Action 类型均为 `control_msgs/action/FollowJointTrajectory`。实际 CLI 输出见 [actions_cli.log](../log/stage5_audit_f611c7zb/regression/actions_cli.log)。

`move_group` 实际加载 OMPL、KDL、关节限制和 `move_group/ExecuteTaskSolutionCapability`。规划后处理包含时间参数化、Ruckig、轨迹验证与显示。关键参数通过定向服务查询确认，见 [move_group 参数](../log/stage5_audit_f611c7zb/regression/move_group_selected_params.json)。

MTC 客户端也实际收到 URDF、SRDF、右臂 KDL、关节加速度限制、`planning_pipelines=[ompl]`、OMPL 插件与后处理参数；并确认其 Group、EEF Link、TCP 与 Fake 等待时间为本次预期值。证据：[MTC 客户端实际参数](../log/stage5_audit_f611c7zb/final_plan/fake_plan_mtc_params.json)。因此，本轮不是只检查 `move_group` 的配置。

## C. 问题、修复与回归记录

| 级别 / 项目 | 证据与根因 | 修复操作 | 回归结果 |
|---|---|---|---|
| P0：真实硬件误启动 | 展开 URDF 与运行时硬件查询均为两侧 GenericSystem；domain 86 启动前为空 | 无需修复；全部测试仅本机发现且使用 Mock | 未发现本轮测试加载真实硬件插件 |
| P1：左夹爪接口不匹配 | 初始 `base.log:52` 显示 MoveIt 创建 GripperCommand；`base.log:282` 实际加载 JointTrajectoryController | 将 v2.0 `moveit_controllers.yaml:54,57` 改为 `FollowJointTrajectory` / `follow_joint_trajectory`；同步集成补丁 | 运行时参数匹配；左夹爪 `+0.2 → 0` 两次执行均 `status=4`、MoveIt error code=1，反馈正确 |
| P1：Fake 冷启动状态等待不足 | `fake_execute.log:319` 初始状态等待超时；同一日志显示 spawner 服务发现重试，广播器尚未就绪 | Fake YAML 中等待时间 `5 → 30 s`；只改变 Fake 配置 | 完整执行通过，随后干净重启的只规划再通过；仍为有界等待，不保证任意慢启动都成功 |
| P2：Launch 运行依赖声明遗漏 | 学习包 Launch 导入/引用了未在其清单声明的包 | 补充 6 个 `exec_depend` | XML 解析、依赖存在性检查、定向构建及实际 Launch 回归通过 |
| P2：MTC 目标链接依赖不显式 | `mtc_pregrasp.cpp` 直接使用 `rclcpp_action`；目标此前依赖间接传递 | 为目标补充 `rclcpp_action`；清单原已声明该依赖 | 重建及完整 MTC 执行通过 |
| P2：MTC 执行插件运行依赖未声明 | 演示依赖 ExecuteTaskSolution capability，但包清单未声明 capability 包 | 增加 `moveit_task_constructor_capabilities` 运行依赖 | 包已存在；运行服务端与完整任务执行通过 |
| P2：退出清理崩溃，未修复 | 成功执行日志 `retry/fake_execute.log:956,958` 后，`:1012,1016` 出现 `move_group` SIGSEGV；栈涉及 MoveItCpp/TEM/rclcpp 清理 | 保存栈和重现；未升级或替换系统 MoveIt/rclcpp | 功能执行通过，干净退出未通过；具体对象生命周期根因尚未定位 |
| P2：全量参数 dump 不完整 | `move_group_params.log` 对可选 `has_jerk_limits` 未初始化参数读取失败，输出空参数树 | 改用只读定向 GetParameters；不修改正常规划参数 | URDF、SRDF、IK、流水线、控制器等所需参数逐项取得 |
| P2：已有日志警告 | 无 Octomap 传感器插件、同名日志发布者等 | 保留日志；本阶段固定场景不依赖 3D 传感器输入 | 未阻断本轮规划和执行，不表述为“日志零警告” |
| 构建路径与沙箱限制 | `/run/host/...` 与当前目录同一文件；沙箱禁止旧测试结果路径写入及 DDS 套接字 | 按权限流程在沙箱外执行必要测试/ROS 验证；没有修改系统权限 | 已有测试通过；不属于缺失 ROS 包或源码编译错误 |
| 其余核心依赖与安装 | 声明包均能解析；头文件、库、CMake 包和安装可执行程序存在 | 无需批量重装或改头文件扩展名 | 两个包构建成功；未观察到 ROS 1/Humble API 编译或链接混用 |

修改文件：

- [MTC CMake](../src/openarm_mtc_pick/CMakeLists.txt)、[MTC package.xml](../src/openarm_mtc_pick/package.xml)、[Fake 配置](../src/openarm_mtc_pick/config/fake_pick_place.yaml)。
- [学习包 package.xml](../src/openarm_moveit_learning/package.xml)。
- [OpenArm 集成补丁](../integration/patches/openarm_ros2.patch)及忽略目录中的对应 v2.0 MoveIt 控制器配置。

控制器补丁已在临时目录对固定上游原文件执行 `git apply --check`，并确认应用结果等于当前修复文件。其余已有补丁内容与用户改动保留。

### 运行结果的边界

成功的完整演示生成 5 条完整解，执行摘要为 **11 个轨迹段、总轨迹时长约 53.306 s**。整个启动、规划、执行及关闭流程约 83.69 s。末态全部受测主动关节绝对值最大约 `9.85e-5 rad`，接近记录的全零初态。

最终查询到 `blue_cube` 世界位置约 `(0.300570, -0.032103, 0.270002) m`，与 `table` 一起留在世界场景，附着集合为空。该位置是查询结果；本轮没有单独计算“选中放置目标与终态物体”的位姿误差，不能将它宣称为物理放置精度。

左臂 Stage 4 实验目标为当前 Joint1 增加 `0.1 rad`，反馈为 `0.100040344 rad`。夹爪回归为独立的开合接口验证，不代表左臂完整抓取已完成。

Launch 外层退出码在子进程失败时也可能为 0：本轮初次失败的 Fake 执行就是如此。因此验收使用子进程错误、最终执行状态和反馈/场景证据，不能只看 `ros2 launch` 的退出码。

## D. 实际系统拓扑

下图主链已通过配置、运行时查询和实际执行交叉验证：

```text
fake_pick_place.launch.py  [execute_fake 默认 false]
  |
  +-> demo.launch.py  [use_fake_hardware=true]
  |     +-> robot_state_publisher -> /tf + /tf_static
  |     +-> controller_manager
  |     |     +-> 两臂 JointTrajectoryController
  |     |     +-> 两夹爪 JointTrajectoryController
  |     |     +-> joint_state_broadcaster
  |     |     +-> 两侧 mock_components/GenericSystem
  |     +-> move_group [ExecuteTaskSolutionCapability + OMPL + KDL]
  |
  +-> fake_blue_cube_pose_publisher
  |     +-> /blue_cube/pose (geometry_msgs/msg/PoseStamped)
  |     +-> /blue_cube/dimensions (geometry_msgs/msg/Vector3Stamped)
  +-> scene_manager -> /apply_planning_scene -> move_group 场景
  +-> dev_pick.launch.py -> pregrasp.launch.py
        +-> openarm_mtc_pregrasp
              +-> 本地模型、IK、OMPL 与 MTC Task 求解
              +-> /get_planning_scene 读取场景
              +-> /execute_task_solution (ExecuteTaskSolution Action)
                       |
                       v
                    move_group
                       |
                  四个控制器的 FollowJointTrajectory Action
                       |
                controller_manager -> GenericSystem
                       |
              joint_state_broadcaster
                       |
                /joint_states (sensor_msgs/msg/JointState)，约 100 Hz
                       +-> robot_state_publisher
                       +-> MoveIt 当前状态监视器
                       +-> MTC 初始状态订阅 / 本轮审计探针
```

`/joint_states` 发布者已确认为 `joint_state_broadcaster`。运行图直接显示 `robot_state_publisher`、MoveIt 私有监视节点及审计探针订阅；一个端点显示 `_NODE_NAME_UNKNOWN_`，不强行给它命名。MTC 订阅关系另由源码 `mtc_pregrasp.cpp:986` 和实际收到初始关节的运行结果确认。

实际 TF 主干为：

```text
world
  -> openarm_body_link0
       +-> openarm_left_base_link -> 左臂连杆 -> openarm_left_ee_base_link
       |                                         -> openarm_left_grasp_frame
       +-> openarm_right_base_link -> 右臂连杆 -> openarm_right_ee_base_link
                                                 -> openarm_right_grasp_frame
```

`blue_cube` 是 PlanningScene 对象，不因有对象 ID 就自动成为 TF 广播中的同名 frame。

**需要区分的静态配置：** `pregrasp.launch.py` 的客户端模型映射写有 `use_fake_hardware=false`，但该 Launch 只启动 MTC 客户端，不启动硬件管理器。实际加载硬件的是 Fake 入口包含的 `demo.launch.py`；本轮其两侧插件均经过运行时核实为 GenericSystem。没有把客户端 URDF 中出现硬件插件名称等同于实际加载驱动。

## E. Stage 5 开工清单与下一步

| 准备项 | 结论 | 说明 |
|---|---|---|
| ROS/Jazzy 与 overlay | 已验证通过 | 每个新终端显式 source；当前非交互 Shell 默认未加载 |
| MTC 客户端模型、IK、流水线与场景 | 已验证通过 | 实际参数查询 + 完整规划 |
| 左右臂 CurrentState | 已验证通过 | 两侧分别运行现有环境检查程序 |
| 左臂关节规划与执行 | 已验证通过 | 复用 Stage 4 程序 |
| MTC 任务执行接口 | 已验证通过 | 现有右臂完整解实际执行 |
| 左夹爪 MoveIt 映射 | 已修复并验证 | 与实际 FollowJointTrajectory 服务端一致 |
| 两侧夹爪开合 | 已验证通过 | 小幅开合及复位，检查最终结果与反馈 |
| 场景增删查询 | 已验证通过 | 使用既有 `scene_obstacle_demo` 并独立查询 |
| 两侧 TCP | 已验证通过 | 运行时 TF 及固定变换已保存 |
| Fake 启动状态等待 | 已修复并验证 | 30 s 有界等待；尚未改造成完整就绪编排机制 |
| 现有右臂 Fake Pick & Place | 已验证通过 | 规划、执行、关节反馈与物体生命周期闭环 |
| 双臂模型/EEF/控制器区分 | 已验证通过 | 左右符号与命名分别确认 |
| 重新启动与干净场景 | 已验证通过 | 多次独立启动；初始关节零值、初始世界场景为空，随后由演示添加目标 |
| 全部进程干净退出 | 存在问题 | `move_group` 退出 SIGSEGV 尚未修复；最终残留节点检查为空 |
| 左臂完整抓放 / 双臂顺序任务 | 待验证 | 属于后续学习实现，未把计划当成果 |

**下一步唯一推荐动作：进入 Stage 5.2，在同一 Fake 基线上只规划并解读已有任务树。** 5.1 所需环境证据已经具备，无需重复全面排查。教学会话先让学员解释 CurrentState、抓取子任务、放置子任务和返回阶段，再进入手写实验。

本轮审计启动的全部实例已停止。以下命令用于**容器内 Bash 终端**，工作空间为 `/home/ashenlad/code/openarm-custom-stack`；确认 domain 86 没有新启动的其他实例后执行这一项演示，不要再额外启动另一套 `demo.launch.py`：

```bash
cd /home/ashenlad/code/openarm-custom-stack
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=86
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
ros2 launch openarm_mtc_pick fake_pick_place.launch.py execute_fake:=false
```

成功判据为存在完整任务解且日志表明未执行，不能只看 Launch 的退出码。规划器具有随机性，5 条解是本轮观察值。

附件中列出的 `src/openarm_mtc_pick/fake_pick_place.launch.py` 路径不存在；实际入口位于 `src/openarm_mtc_pick/launch/`。`docs/05_stage5_plan.md` 存在，计划中的 `src/openarm_mtc_learning/` 尚不存在。本轮实际 Stage 4 总结文件名为 `docs/04_stage4_summary.md`，后续教学应按真实路径读取。

## F. Stage 5 环境交接摘要

当前为 CachyOS 宿主的 `ros-jazzy` 容器，Ubuntu 24.04.4 / Jazzy / MoveIt 2.12.4 / 本工作空间 MTC 0.1.5，GCC 13.3.0，C++17。OpenArm v2.0 双臂以 `world` 为根，两侧实际硬件均为 GenericSystem。四个运动控制器均使用 FollowJointTrajectory，MTC 使用 `/execute_task_solution` Action。已完成两包构建、已有 MTC 测试、左右 CurrentState、左臂关节运动、两侧夹爪开合、场景增删、TCP TF 和现有右臂完整 Fake 抓放。已修正左夹爪映射、Fake 状态等待时间及依赖声明；保留原有上游修改。遗留 `move_group` 退出清理崩溃，功能成功不能等同干净退出。实例现已停止、domain 86 查询为空。下一微任务为 Stage 5.2 只规划演示及任务树解读，不重新全面审计，也不提前实现完整学习版抓放。

## 证据索引

日志根目录：[stage5_audit_f611c7zb](../log/stage5_audit_f611c7zb/)。主要文件：

| 文件 | 用途 |
|---|---|
| `system.txt` / `baseline.txt` / `linkage.txt` | 系统版本、容器、包前缀、上游状态和动态链接 |
| `static_audit.json` / `fake_robot.urdf` | 真实文件存在性、模型展开、关节/EEF/限位与路径别名 |
| `base.log` / `controllers.log` / `hardware.log` | 初始运行基线与左夹爪配置错误证据 |
| `environment_check.log` | 右臂 CurrentState 检查 |
| `fake_plan.log` / `fake_plan_result.json` | 初次只规划及无关节运动证据 |
| `fake_execute.log` | 首次执行尝试的状态等待超时；不应作为成功样例 |
| `retry/fake_execute.log` / `retry/fake_execute_result.json` | 完整执行成功与退出崩溃同时记录 |
| `retry/fake_execute_scenes.json` | 独立查询的物体附着、分离与终态 |
| `regression/interfaces_result.json` / `regression/left_joint.log` | 左右夹爪与左臂运动回归 |
| `regression/scene_add.log` / `regression/scene_remove.log` | 场景增删；独立查询结果在 interfaces_result 中 |
| `regression/left_environment.log` / `regression/tcp_tf.json` | 左臂 CurrentState 与两侧 TCP 实际 TF |
| `regression/actions_cli.log` / `regression/move_group_selected_params.json` | Action 实际名称类型与服务端参数 |
| `final_plan/fake_plan.log` / `final_plan/fake_plan_mtc_params.json` | 最终只规划回归及 MTC 客户端实际参数 |
| `build.log` / `tests_retry.log` / `test_results.log` | 定向构建与已有测试结果 |
| `preservation.txt` / `upstream_change_delta.diff` | 用户已有修改保留与本轮上游最小变更 |
| `cleanup.json` | domain 86 无残留节点 |

审计辅助脚本仅保存在忽略的日志目录，没有新增学习包或完整 Pick & Place 实现。
