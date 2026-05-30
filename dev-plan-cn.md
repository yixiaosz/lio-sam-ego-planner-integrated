# RMUA2026 集成开发计划（FAST-LIVO2 版）

> 本文档替代 `dev-plan-cn.md.bak`，作为 Simulator → FAST-LIVO2 → EGO-Planner → MPC 全链路的动态集成计划。

---

## 1. 目标

为 RMUA 2026 AirSim 四旋翼仿真器构建一套完整的自主导航管线：

1. **感知 / SLAM**：将仿真器的 LiDAR、IMU 与左前相机数据输入 **FAST-LIVO2**（Full LIVO 模式），实时建图与定位。
2. **规划**：将 FAST-LIVO2 输出的里程计与配准点云输入 **EGO-Planner**，生成无碰撞轨迹。
3. **控制**：将 EGO-Planner 生成的轨迹输入 **MPC 控制器**，输出电机 PWM 指令。

> **注意**：真值话题 `/airsim_node/drone_1/debug/pose_gt` 仅用于本地调试，**正式比赛不可用**。

---

## 2. 话题接口对齐表

| 来源 | 话题 | 消息类型 | 坐标系 / 备注 |
|------|------|----------|---------------|
| **仿真器** | `/airsim_node/drone_1/lidar` | `sensor_msgs/PointCloud2` | NED，传感器局部坐标系。标准 PointCloud2（无 ring/time）。 |
| **仿真器** | `/airsim_node/drone_1/imu/imu` | `sensor_msgs/Imu` | NED 坐标系。 |
| **仿真器** | `/airsim_node/drone_1/front_left/Scene` | `sensor_msgs/Image` | NED，左前视相机。作为 FAST-LIVO2 LIVO 模式的单目输入。 |
| **适配器**（输入） | — | — | 接收上述仿真器话题。 |
| **适配器**（输出） | `/sim/lidar` | `sensor_msgs/PointCloud2` | ENU，LiDAR 局部坐标系。标准 PointCloud2，无需注入 ring/time。 |
| **适配器**（输出） | `/sim/imu` | `sensor_msgs/Imu` | ENU，符合 REP-105 规范。 |
| **适配器**（输出） | `/sim/camera` | `sensor_msgs/Image` | ENU 对齐的图像流，供 FAST-LIVO2 视觉线程使用。 |
| **FAST-LIVO2**（输入） | `lid_topic` | `sensor_msgs/PointCloud2` | 可配置。标准 PointCloud2，使用 `lidar_type: 3`。 |
| **FAST-LIVO2**（输入） | `imu_topic` | `sensor_msgs/Imu` | 可配置。 |
| **FAST-LIVO2**（输入） | `img_topic` | `sensor_msgs/Image` | 可配置。单目左前视图像。 |
| **FAST-LIVO2**（输出） | `/aft_mapped_to_init` | `nav_msgs/Odometry` | `camera_init` 坐标系，ENU，EKF 融合位姿（LiDAR 频率）。 |
| **FAST-LIVO2**（输出） | `/imu_prop_odom` | `nav_msgs/Odometry` | `camera_init` 坐标系，ENU，高频 IMU 传播位姿。 |
| **FAST-LIVO2**（输出） | `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` 坐标系，ENU，去畸变稠密点云。 |
| **FAST-LIVO2**（输出） | `/path` | `nav_msgs/Path` | 轨迹历史。 |
| **FAST-LIVO2**（输出） | `/rgb_img` | `sensor_msgs/Image` | 带视觉跟踪叠加的重发布图像（LIVO 激活时）。 |
| **桥接**（输出） | `odom_topic` | `nav_msgs/Odometry` | `world` 坐标系，ENU，连续位姿。由 `/aft_mapped_to_init` 重命名而来。 |
| **桥接**（输出） | `cloud_topic` | `sensor_msgs/PointCloud2` | `world` 坐标系，ENU，障碍物地图。由 `/cloud_registered` 中继。 |
| **EGO-Planner**（输出） | *(自定义轨迹消息)* | 待定 | B-spline 轨迹（位置、速度、加速度、偏航）。 |
| **MPC**（输入） | *(自定义轨迹消息)* | 待定 | 以 100 Hz 采样的参考轨迹。 |
| **MPC**（输出） | `/airsim_node/drone_1/rotor_pwm_cmd` | `airsim_ros/RotorPWM` | 直接电机控制指令。 |

---

## 3. 管线阶段与适配层

### 3.1 仿真器 → FAST-LIVO2 适配器 (`sim_sensor_adapter`)

**状态**：❌ **尚不存在。这是当前最关键缺失模块。**

在 FAST-LIVO2 能初始化之前，该适配节点必须完成以下转换：

1. **坐标系转换 NED → ENU**
   - 对点云、IMU 线加速度 / 角速度以及图像消息头中的 `frame_id` 施加静态旋转。
   - 对于图像流，像素内容不变；仅将消息头中的 `frame_id` 重写为 ENU 对齐的相机坐标系。
2. **重发布 LiDAR**
   - 订阅 `/airsim_node/drone_1/lidar`。
   - 将点坐标从 NED 变换到 ENU。
   - 以标准 `sensor_msgs/PointCloud2` 格式发布到 `/sim/lidar`。
   - **无需注入 ring 或 time 字段**，因为 FAST-LIVO2 将使用 `preprocess.lidar_type: 3`（标准 PointCloud2 处理器）。
3. **重发布 IMU**
   - 订阅 `/airsim_node/drone_1/imu/imu`。
   - 将线加速度与角速度从 NED 旋转到 ENU。
   - 发布到 `/sim/imu`。
4. **重发布相机**
   - 订阅 `/airsim_node/drone_1/front_left/Scene`。
   - 将消息头 `frame_id` 重写为 ENU 对齐的相机坐标系。
   - 发布到 `/sim/camera`。
5. **时间同步**
   - 以 **IMU 消息时间戳** 作为主时钟参考，而不是 `ros::Time::now()`，避免主机与仿真器时钟偏移。
6. **相机-雷达-IMU 外参**
   - 仿真器提供确定性的传感器位姿。适配器无需估计外参，但 FAST-LIVO2 的 YAML 配置必须包含：
     - `extrinsic_T` / `extrinsic_R`：雷达到 IMU 的变换
     - `Rcl` / `Pcl`：相机到雷达的旋转和平移
     - 相机内参：`fx`、`fy`、`cx`、`cy`、畸变模型参数

### 3.2 FAST-LIVO2 → EGO-Planner 桥接 (`fastlivo2_to_ego_bridge`)

**状态**：❌ **尚不存在。**

FAST-LIVO2 在 `camera_init` 坐标系下输出里程计与配准点云。EGO-Planner 期望输入在 `world` / ENU 坐标系下。

**桥接节点职责：**
1. 订阅 `/aft_mapped_to_init` 与 `/cloud_registered`。
2. 将两条消息中的 `frame_id` 从 `camera_init` 重命名为 `world`。
3. 将里程计发布到 EGO-Planner 配置的 `odom_topic`。
4. 将点云发布到 EGO-Planner 配置的 `cloud_topic`。
5. 可选：订阅 `/imu_prop_odom`，将其作为高频 fallback 或平滑输入，供 EGO-Planner 内部状态预测器使用。
6. **TF 广播**：确保 `world` → `base_link` 的 TF 被一致发布，要么在桥接中重发布 FAST-LIVO2 的 TF（使用新 frame 名），要么让 EGO-Planner 直接消费里程计消息。

**点云密度**：若 `/cloud_registered` 对 EGO-Planner 的 `plan_env` 过于稠密，可在桥接与 EGO-Planner 之间插入 VoxelGrid 降采样节点。

### 3.3 EGO-Planner → MPC 接口 (`ego_to_mpc_bridge`)

**状态**：❌ **尚未定义。需团队就消息格式达成一致。**

EGO-Planner 生成 B-spline 轨迹（位置、速度、加速度、偏航、偏航角速度）。MPC 组需要稳定的参考流。

**建议的消息接口**（拟新增至 `airsim_ros` 或 `quadrotor_msgs`）：

```yaml
# Trajectory.msg
Header header
TrajectoryPoint[] points

# TrajectoryPoint
float64 time_from_start
float64 x, y, z
float64 vx, vy, vz
float64 ax, ay, az
float64 yaw
float64 yaw_dot
```

**桥接节点职责：**
1. 订阅 EGO-Planner 轨迹输出。
2. 缓存最新有效轨迹。
3. 在当前时刻采样轨迹，生成参考状态。
4. 以 MPC 控制频率（100 Hz）发布。
5. 处理边界情况：尚无轨迹时发布悬停设定点；轨迹过期时请求重规划。

### 3.4 MPC → 仿真器

**状态**：✅ **`controller_test` 中已部分实现。**

- 现有 `controllerTest.cpp` 从文本文件加载航点并调用 `rpg_mpc::MpcController::run()`。
- 一旦轨迹消息定义完成，将文件加载器替换为对应话题的订阅器。
- 保留现有 `RotorPWM` 发布器与电机索引映射（`0:RF, 1:LR, 2:LF, 3:RR`）。

---

## 4. 旧版双目视觉里程计包 (`odometry`)

**状态**：⚠️ ** sidelines（保留但不参与主链路）。保留在工作区中，但不在比赛启动路径内。**

现有的 `odometry` 包使用左前 + 右前相机、光流与 g2o 进行双目视觉里程计。其架构本质上是一个调试/教学节点：

- 订阅 `/airsim_node/drone_1/pose`（仿真器真值/局部位姿），因此不具备独立竞赛能力。
- 仅发布调试输出（`/display_image`、`/cur_frame_pcl`、`/local_map_pcl`），不发布里程计消息。
- **无法向 FAST-LIVO2 提供输入**，因为 FAST-LIVO2 的 LIVO 模式直接消费原始单目图像并在内部执行直接光度对齐。

**处置方案：**
- **源代码**：保留在 `src/odometry/` 中，供参考和本地调试对比使用。
- **构建**：从 `catkin_make` 白名单和 `setup.bash` 自动启动中移除。
- **运行时**：不在比赛管线中启动。FAST-LIVO2 直接订阅 `front_left/Scene`。

---

## 5. FAST-LIVO2 配置需求

需为仿真器新建配置文件（例如 `fast_livo2_sim.yaml`）。关键参数如下：

**common:**
- `lid_topic`: `/sim/lidar`
- `imu_topic`: `/sim/imu`
- `img_topic`: `/sim/camera`
- `img_en`: `1`（启用视觉-惯性线程）
- `lidar_en`: `1`
- `slam_mode`: `2`（LIVO）

**preprocess:**
- `lidar_type`: `3`（标准 PointCloud2 / Ouster 风格处理器）
- `point_filter_num`: `1`
- `blind`: `0.5`

**extrinsics:**
- `extrinsic_T` / `extrinsic_R`：雷达到 IMU 的变换（由仿真器决定）
- `Rcl` / `Pcl`：相机到雷达的旋转和平移（由仿真器决定）

**camera:**
- 必须为 Vikit 填充相机模型类型与内参（AirSim Scene 相机预计使用 Pinhole + Radtan）。
- 若非零，需提供畸变系数。

**publish:**
- `scan_publish_en`: `true`
- `dense_publish_en`: `true`
- `path_en`: `true`（可选，调试用）

---

## 6. 依赖与构建系统变更

### 需新增依赖
1. **FAST-LIVO2** 克隆到 `RMUA2026-dev/basic_dev/src/`
2. **Vikit** (`xuankuzcr/rpg_vikit`) 克隆到 `src/` —— 需作为 catkin 包编译
3. **Sophus**（系统级安装，指定 commit `a621ff`）

### 可移除依赖（若完全弃用 LIO-SAM）
- `libgtsam-dev`
- `libgtsam-unstable-dev`
- `LIO-SAM-raw/` 参考包（保留目录作为上游参考，但不在比赛工作区中编译）

### 编译顺序
```bash
catkin_make --only-pkg-with-deps fast_livo
# 或全工作区编译
```

---

## 7. 开发顺序

1. **`sim_sensor_adapter`** —— 影响最大。仅需 NED→ENU 转换，无需 ring/time 注入。
2. **FAST-LIVO2 参数调优** —— 编写 `fast_livo2_sim.yaml`，填入相机内参与仿真器外参。对照 `debug/pose_gt` 验证。调优 `acc_cov`、`gyr_cov`、`blind`、`point_filter_num`。
3. **`fastlivo2_to_ego_bridge`** —— frame 重命名（`camera_init` → `world`）、话题中继、可选 VoxelGrid 滤波。
4. **FAST-LIVO2 → EGO-Planner 对接** —— 重映射话题，验证局部地图正常构建，确认规划器能无崩溃输出轨迹。
5. **冻结轨迹消息定义** —— 与 MPC teammate 就 `.msg` 定义达成一致，提交至 `airsim_ros` 或 `quadrotor_msgs`。
6. **`ego_to_mpc_bridge`** —— 将 EGO-Planner 轨迹采样为已约定的消息格式。MPC 组可并行使用虚拟轨迹发布器进行开发。
7. **全栈联调** —— 在仿真器中运行完整管线，调试时序 / TF / frame_id 问题。
8. **Docker 打包** —— 更新 `setup.bash` 自动启动完整管线，构建提交镜像并导出 `test.tar`。

---

## 8. 架构图

```
┌─────────────┐      ┌──────────────────┐      ┌──────────────┐
│  RMUA Sim   │─────▶│ sim_sensor_adapter│─────▶│  FAST-LIVO2  │
│  (NED)      │      │ (NED→ENU, 无需    │      │   (ENU)      │
│             │      │  ring/time 注入)  │      │  LIVO 模式   │
└─────────────┘      └──────────────────┘      └──────┬───────┘
     front_left/Scene                                │
                                                      │
                           ┌──────────────────────────┘
                           ▼
                    ┌─────────────────┐
                    │fastlivo2_to_ego_│
                    │     bridge      │  (camera_init → world)
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  EGO-Planner    │
                    │  (plan_manage)  │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  ego_to_mpc_    │
                    │     bridge      │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  MPC Tracker    │
                    │  (controller)   │
                    └────────┬────────┘
                             ▼
                    ┌──────────────┐
                    │  Simulator   │
                    │ (RotorPWM)   │
                    └──────────────┘

[Sidelined] odometry（双目 VO 调试节点，不启动）
```

---

## 9. 待你决策的开放问题

以下选择将直接影响实现方式。请在开始编码前确认：

### 决策 A：轨迹消息标准
EGO-Planner 如何向 MPC 发送轨迹？

- **选项 1 — 新建自定义消息**（`Trajectory` / `TrajectoryPoint`），放在 `airsim_ros` 中。干净、专用，但需双方共同约定 `.msg` 文件。**（推荐）**
- **选项 2 — 复用现有消息**。若 `ego-planner-raw` 或 `rpg_quadrotor_common` 中已有合适消息，可直接使用。启动快，但可能有多余字段或缺少必要字段。
- **选项 3 — 使用 `nav_msgs/Path` + 自定义话题约定**。简单，但会丢失速度/加速度/偏航信息，不足以支持 MPC。

> **我的建议**：选项 1。定义一个包含 `time_from_start`、位置、速度、加速度、偏航及偏航角速度的简洁 `TrajectoryPoint[]` 消息。

### 决策 B：EGO-Planner 的局部地图来源
EGO-Planner 消费哪路点云？

- **选项 1 — 直接喂入 FAST-LIVO2 的 `/cloud_registered`**。最简单。EGO-Planner 的 `plan_env` 会自行构建局部地图。风险：点云可能过密或过疏，取决于调参。**（推荐）**
- **选项 2 — 先对 `/cloud_registered` 做 VoxelGrid 降采样**。增加一个滤波节点，但能保证 `plan_env` 的点密度上限。
- **选项 3 — LiDAR 与双目深度融合**。更鲁棒，但复杂度显著增加，对当前比赛环境可能没有必要。

> **我的建议**：先选选项 1。若 EGO-Planner 的 CPU 负载过高或局部地图表现不佳，再追加 VoxelGrid 节点（选项 2）作为无侵入替换。

### 决策 C：起飞与任务启动逻辑
谁触发初始起飞？规划器何时介入？

- **选项 1 — `setup.bash` 调用一次 `/takeoff` 服务，等待稳定悬停后再启动所有节点**。简单，但所有节点在容器内同时启动，可能错过最初几秒传感器数据。
- **选项 2 — 所有节点立即启动（包括 `sim_sensor_adapter` 与 FAST-LIVO2）。一个轻量级状态机节点等待 `/airsim_node/initial_pose`，调用 takeoff，随后通知 EGO-Planner 开始规划**。更鲁棒，但需编写一个小型任务管理节点。**（推荐）**

> **我的建议**：选项 2。`imu_gps_odometry` 节点已具备阻塞等待 `/airsim_node/initial_pose` 的模式；将其扩展为微型任务管理器是自然的做法，可避免竞态条件。

### 决策 D：FAST-LIVO2 失效 / 漂移时的降级策略
若 FAST-LIVO2 丢跟踪或漂移，如何处理？

- **选项 1 — 无降级策略。FAST-LIVO2 失效则任务结束**。代码路径最简单。若比赛允许多次运行或环境对 SLAM 友好，则可接受。**（第一里程碑推荐）**
- **选项 2 — 降级到 ESKF 里程计 (`/eskf_odom`)**。现有 `imu_gps_odometry` 包已发布该话题。若 FAST-LIVO2 里程计停止更新，EGO-Planner 可切换至该来源。增加复杂度，但提高鲁棒性。

> **我的建议**：第一里程碑选选项 1。仅在联调中证明 FAST-LIVO2 在特定仿真环境中存在可靠性问题时，再引入选项 2。

---

## 10. 已知陷阱

1. **硬编码路径**：`controllerTest.cpp` 中包含绝对轨迹文件路径。在 Docker 提交前必须改为参数化加载。
2. **时钟偏移**：不要依赖 `ros::Time::now()`。以 IMU 消息时间戳作为主时钟。
3. **Frame ID 不匹配**：TF 或消息 `frame_id` 中 `camera_init`、`world`、`odom` 的不一致将静默破坏 FAST-LIVO2 或 EGO-Planner。
4. **提交镜像禁止 GUI**：最终 Docker 镜像不能启动 RViz 或任何 X11 程序。若比赛服务器磁盘空间允许，可启用 PCD/TUM 保存以供赛后分析。
5. **EGO-Planner 需要初始目标**：确保任务管理器或启动文件在首个航点附近提供默认目标，或订阅 `/airsim_node/end_goal`。
6. **Sophus 版本锁定**：FAST-LIVO2 + Vikit 要求 Sophus commit `a621ff`。新版本 Sophus 会破坏编译。
7. **相机外参精度**：LIVO 模式对雷达-相机外参（`Rcl` / `Pcl`）敏感。仿真器外参是确定性的，但任何手动抄写错误都会导致视觉-雷达错配与漂移。
8. **标准 PointCloud2 模式的权衡**：使用 `lidar_type: 3` 会失去 Livox 原生的每点 `offset_time` 元数据。若仿真器 LiDAR  closely 模拟 MID360 扫描模式且观察到漂移，可考虑构建 `PointCloud2 → CustomMsg` 转换器以使用 `lidar_type: 1`。

---

*最后更新：2026-05-30*
