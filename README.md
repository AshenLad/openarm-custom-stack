# OpenArm Custom Stack

面向 OpenArm v2.0 与 Intel RealSense D435i 的 RGB-D 视觉抓放示例。项目把箱体/物体检测、动态 PlanningScene、MoveIt Task Constructor（MTC）完整抓放任务，以及夹爪 torque 监测和重力补偿集成为一次视觉事务。

> 当前实现以右臂、俯视固定相机、箱顶物体和垂直夹取为主。真机具有碰撞和夹伤风险；请先完成 fake/Gazebo 验证，并自行标定相机外参、机械臂零位与夹爪 torque。仓库没有提供可直接套用的实机外参或 torque 阈值。

## 工作原理

```text
D435i RGB + aligned depth
        │
        ├─ 箱体检测：白色证据 + RGB-D 点云 + 支撑/顶面拟合
        └─ 物体检测：箱顶 footprint 内的通用 3-D 聚类与 OBB
                         │
                         ▼
              world 坐标下的稳定视觉 LOCK
                         │
                         ▼
             PlanningScene 动态碰撞几何快照
                         │
                         ▼
        MTC 一次规划完整 Pick → Place → Return 任务
                         │
                执行前新鲜度/位姿复核
                         │
                         ▼
               立即执行刚选中的同一条解
```

正式流程不是“先 plan-only，过一会儿再重新规划执行”。`pick.launch.py` 会等待稳定视觉结果、提交同一份场景快照、完整规划、复核视觉，然后立即执行同一条解，以减少相机抖动、时间差和随机 IK 导致的不一致。

完整任务为：记录初始关节 → 打开夹爪 → 预抓取 → 垂直下降 → 动态孔径夹紧 → Attach → 垂直抬升 → 箱顶候选放置 → 松开 → Detach → 垂直撤离 → 收拢空夹爪 → 返回初始关节。

主要设计点：

- 箱体和物体均来自实时 RGB-D，不在 MTC 中写死实物位置或尺寸；
- 稳定 LOCK 会冻结一次事务的几何，真实变化需连续多帧确认；
- 抓取高度由物体尺寸、官方 grasp TCP 和指尖几何动态计算；
- 放置高度由实测箱顶、物体高度和安全净空动态计算，避免指尖或物体继续压向箱面；
- 物体 yaw 对称候选、箱顶多个位置和多个放置 yaw 一起交给 IK/碰撞检查筛选；
- 执行前检查目标/箱体是否过期、移动或改变尺寸，变化超限就拒绝执行并要求重规划；
- `monitor` 只发布和记录电机侧 torque；`enforce` 会改变夹爪指令，必须在实机标定后才可启用；
- fake 场景支持固定种子随机化箱体、物体位置、尺寸、yaw 与 Z 测量误差，避免只验证单一场景。

## 仓库内容

```text
openarm-custom-stack/
├── src/
│   ├── openarm_aruco_vision/   # ArUco 标定辅助、箱体/物体 RGB-D 检测
│   └── openarm_mtc_pick/       # PlanningScene、MTC、正式与仿真入口
├── integration/
│   ├── patches/                # 针对固定上游 commit 的最小补丁
│   └── openarm_ros2_new_files/ # torque、重力补偿和接触仿真新增文件
├── upstream.repos              # 上游仓库与可复现 commit
├── LICENSE
└── NOTICE
```

仓库不复制 OpenArm、MoveIt Task Constructor、OpenArm CAN 或 RealSense 的完整第三方源码。`upstream.repos` 会从原作者仓库获取固定版本；本项目只发布原创 ROS 2 包、必要集成补丁和新增文件。第三方项目仍受各自许可证约束。

## 环境与硬件

已验证的主要环境：

- Ubuntu 24.04 / ROS 2 Jazzy；
- OpenArm v2.0 双臂描述与 `ros2_control`，抓取使用右臂；
- MoveIt 2 与 MoveIt Task Constructor；
- Intel RealSense D435i，彩色图、对齐深度和 CameraInfo；
- 固定相机的 `world → global_camera_link` 外参；
- OpenArm 夹爪使用 `FollowJointTrajectory` 控制接口。

算法本身不依赖某个相机序列号。默认话题为：

```text
/camera/global_camera/color/image_raw
/camera/global_camera/aligned_depth_to_color/image_raw
/camera/global_camera/color/camera_info
```

如果相机命名不同，请修改 `src/openarm_aruco_vision/config/*.yaml`。

## 获取源码和构建

先安装 ROS 2 Jazzy、MoveIt 2、RealSense ROS 2 驱动、Gazebo/ros_gz（需要仿真时）、`vcstool`、`rosdep` 和 `colcon`。然后：

```bash
git clone https://github.com/Asukaandmeaaa/openarm-custom-stack.git
cd openarm-custom-stack

# 下载固定版本的第三方依赖；不会把它们提交进本仓库
vcs import . < upstream.repos

# 应用本项目对 OpenArm 上游的集成修改
git -C third_party/openarm_description apply --check \
  "$PWD/integration/patches/openarm_description.patch"
git -C third_party/openarm_description apply \
  "$PWD/integration/patches/openarm_description.patch"

git -C third_party/openarm_ros2 apply --check \
  "$PWD/integration/patches/openarm_ros2.patch"
git -C third_party/openarm_ros2 apply \
  "$PWD/integration/patches/openarm_ros2.patch"
cp -a integration/openarm_ros2_new_files/. third_party/openarm_ros2/

source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src third_party --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

补丁只保证应用于 `upstream.repos` 中记录的 commit。若要跟随更新后的 OpenArm 上游，请先人工复核 URDF、硬件接口和关节限制，不要强行使用 `git apply --reject`。

## 先做无硬件验证

### 1. 单元测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select openarm_aruco_vision openarm_mtc_pick openarm_hardware
colcon test-result --verbose
```

### 2. 一条命令验证正式编排器

该入口硬编码 fake controllers，不会访问 CAN：

```bash
ros2 launch openarm_mtc_pick fake_production_demo.launch.py
```

### 3. 固定场景 plan-only

```bash
ros2 launch openarm_mtc_pick fake_pick_place.launch.py execute_fake:=false
```

可以显式改变场景，例如：

```bash
ros2 launch openarm_mtc_pick fake_pick_place.launch.py \
  execute_fake:=false \
  cube_x:=0.330 cube_y:=0.020 cube_yaw:=0.40 \
  box_x:=0.350 box_y:=0.020 box_yaw:=0.10
```

### 4. 随机场景回归

```bash
ros2 run openarm_mtc_pick randomized_fake_regression.py \
  --scenarios 20 --seed 20260821
```

脚本随机化箱体 XY/Z、长宽高、yaw，物体 XY/Z、长宽高、yaw 以及独立的视觉高度误差。`--dry-run` 只打印场景，`--only N` 可复现某个失败编号。默认使用隔离 DDS domain 和 localhost discovery，避免发现真实机械臂。

随机回归的目的就是暴露工作空间、IK 或碰撞边界，不保证每个随机样本都可达。任一场景失败时脚本会返回非零、保存日志和精确复现命令；应分析失败阶段或收窄经过实机定义的采样范围，不要把失败样本静默算作通过。

### 5. Gazebo RGB-D 闭环

Gazebo 只发布 RGB、对齐深度和 CameraInfo，碰撞体仍必须由同一套视觉节点恢复：

```bash
# 默认只规划
ros2 launch openarm_mtc_pick sim_rgbd_pick.launch.py gazebo_gui:=false

# 使用 fake controllers 完整执行
ros2 launch openarm_mtc_pick sim_rgbd_pick.launch.py \
  gazebo_gui:=false execute_fake:=true
```

## 实机前必须完成的标定

1. 检查 OpenArm 机械零位与模型是否一致，尤其是 wrist 和 joint4；
2. 用你自己的 D435i 安装姿态完成手眼标定；
3. 发布且核对唯一的 `world → global_camera_link`；
4. 检查箱体顶面、物体中心和尺寸在 `world` 中是否与实测一致；
5. 从低速、空载、`gripper_contact_mode:=monitor` 开始采 torque；
6. fake、随机回归和 plan-only 均通过后，再进行有人值守的完整实机测试。

手眼标定文件是数据存储，不会自动发布 TF。下面所有 `<...>` 都必须替换为本机标定结果，不能使用其他机器或仓库作者的数值：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x <tx_m> --y <ty_m> --z <tz_m> \
  --qx <qx> --qy <qy> --qz <qz> --qw <qw> \
  --frame-id world --child-frame-id global_camera_link

timeout 10s ros2 run tf2_ros tf2_echo world global_camera_link
timeout 10s ros2 run tf2_ros tf2_echo \
  world global_camera_color_optical_frame
```

## 实机完整启动流程

每个终端先执行：

```bash
cd "$HOME/openarm-custom-stack"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### 终端 1：D435i

如果只有一台相机，可省略 `serial_no`；多相机时填本机序列号：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=global_camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
```

### 终端 2：手眼 TF

```bash
ros2 run tf2_ros static_transform_publisher \
  --x <tx_m> --y <ty_m> --z <tz_m> \
  --qx <qx> --qy <qy> --qz <qz> --qw <qw> \
  --frame-id world --child-frame-id global_camera_link
```

另开一次性检查：

```bash
timeout 10s ros2 run tf2_ros tf2_echo world global_camera_link
timeout 10s ros2 topic hz /camera/global_camera/color/image_raw
timeout 10s ros2 topic hz \
  /camera/global_camera/aligned_depth_to_color/image_raw
```

### 终端 3：真实 ros2_control + MoveIt

下面的 CAN 映射只是双臂示例，请按自己的布线修改。硬件插件激活可能立即使能并运动，急停必须可用：

```bash
mkdir -p "$HOME/openarm_logs"
STAMP=$(date +%Y%m%d_%H%M%S)

ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v2.0 \
  use_fake_hardware:=false \
  robot_controller:=joint_trajectory_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  gravity_compensation:=true \
  gravity_scale:=1.0 \
  gripper_contact_mode:=monitor \
  gripper_torque_threshold:=0.0 \
  gripper_torque_release_threshold:=0.0 \
  gripper_hard_torque_limit:=0.0 \
  gripper_kp:=5.0 \
  gripper_kd:=0.1 \
  gripper_hold_kp_scale:=1.0 \
  2>&1 | tee "$HOME/openarm_logs/moveit_monitor_${STAMP}.log"
```

应看到重力补偿、`gripper_contact_mode=monitor` 和 Ruckig adapter 已加载。RViz 在正式 demo 中只用于观察 PlanningScene 和轨迹，不要从 RViz 再发送另一条执行命令。

### 终端 4：记录夹爪 torque

```bash
mkdir -p "$HOME/openarm_logs"
STAMP=$(date +%Y%m%d_%H%M%S)
ros2 topic echo /right_gripper/motor_torque \
  | tee "$HOME/openarm_logs/right_gripper_torque_${STAMP}.log"
```

### 终端 5：唯一正式抓取命令

保持人员可随时急停；先确认箱体、物体、相机和机械臂初始状态正确：

```bash
mkdir -p "$HOME/openarm_logs"
STAMP=$(date +%Y%m%d_%H%M%S)
ros2 launch openarm_mtc_pick pick.launch.py \
  2>&1 | tee "$HOME/openarm_logs/production_full_${STAMP}.log"
```

`pick.launch.py` 会自行启动组合视觉和 `scene_manager`。正式运行时不要另外启动 `rgbd_scene_detection.launch.py`、单独 detector 或 `scene_manager.launch.py`，否则会重复节点和重复发布。

成功结束标志：

```text
OPENARM PICK DEMO COMPLETE - full visual pick/place/return succeeded
```

## 调试图像与状态检查

检测使用原始分辨率，只有 debug 图像会压缩到配置的最大宽高。主机加入相同 ROS domain 后可查看：

```bash
rqt_image_view /box/debug_image
rqt_image_view /blue_cube/debug_image
```

也可以检查整体几何：

```bash
ros2 topic echo /box/pose --once
ros2 topic echo /box/dimensions --once
ros2 topic echo /box/top_height --once
ros2 topic echo /blue_cube/pose --once
ros2 topic echo /blue_cube/dimensions --once
ros2 control list_controllers
```

物体底面高度 `object_z - object_height / 2` 应接近箱顶；尺寸应与实物同量级；XY 应位于当前旋转箱体 footprint 内。视觉数值不对时先修相机内参、深度对齐、TF 或检测参数，不要通过修改 MTC 目标来补偿。

## 常用规划参数

参数位于 `src/openarm_mtc_pick/config/pick.yaml`：

| 参数 | 含义 | 对规划/执行的影响 |
|---|---|---|
| `velocity_scaling` | 轨迹最大速度缩放 | 越小执行越慢，通常不改变几何可达性，但会改变时间参数化和跟踪表现 |
| `acceleration_scaling` | 轨迹最大加速度缩放 | 越小启停更缓，过小会增加总时长 |
| `support_clearance` | 指尖最低碰撞几何高于支撑碰撞顶面的净空 | 增大可减少碰箱，过大会让抓取点过高、夹持面积不足或无 IK |
| `surface_clearance` | 放置时物体底面高于实测箱顶的释放间隙 | 增大可避免压箱，过大会增加跌落和位姿偏差 |
| `collision_padding_top` | 仅向箱体碰撞模型顶部增加的安全层 | 增大使规划更保守；物理放置高度计算会扣除该 padding，避免把视觉箱高伪造为更高 |

这些量都会直接或间接影响规划。不要只为“出解”降低安全净空；应先验证视觉顶面误差、TCP、碰撞模型和机械零位。

## Torque 模式

- `disabled`：保留传统夹爪行为；
- `monitor`：滤波、记录和发布真实电机侧 torque，不覆盖夹爪命令；
- `enforce`：确认接触或硬 torque 限制后锁存保持位置，会干预命令。

torque 是电机侧 Nm，不是指尖牛顿力。不同夹爪、物体、传动摩擦和安装状态的空载 bias 都不同。首次使用只运行 `monitor`，采集空载、接触、夹持、抬升和释放全过程，再确定阈值、确认周期、保持增益和硬限制。

## 已知边界

- 当前是 snapshot-and-replan，不是运动中的视觉伺服；目标在执行期间移动不会被跟踪；
- 默认检测器先找白色箱顶，再在其 footprint 内检测通用凸起物；其他台面外观需要重新配置或替换箱体 detector；
- 当前正式策略是垂直抓取；工作空间边缘、腕关节极限或遮挡场景可能没有完整解；
- `Final: solutions > 0` 只代表整个 MTC 任务存在候选解，不代表实机标定、夹持力或视觉绝对精度正确；
- 更换相机位置、机械臂底座、夹爪、箱体或控制增益后必须重新验证；
- 本仓库不提供安全认证，不能用于无人值守或人员可进入的工业生产环境。

## License 与上游归属

本项目原创代码使用 Apache-2.0。OpenArm、OpenArm CAN、MoveIt Task Constructor、RealSense ROS 等属于各自作者；补丁是针对其 Apache-2.0 代码的修改，详细基线和归属见 `NOTICE` 与 `integration/README.md`。
