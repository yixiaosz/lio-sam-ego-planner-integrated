# RMUA2026 集成开发计划

> 本文档替代 `project-description-short.md`，作为 Simulator → LIO-SAM → EGO-Planner → MPC 全链路的动态集成计划。

---

## 1. 目标

为 RMUA 2026 AirSim 四旋翼仿真器构建一套完整的自主导航管线：

1. **感知 / SLAM**：将仿真器的 LiDAR 与 IMU 数据输入 **LIO-SAM**，实时建图与定位。
2. **规划**：将 LIO-SAM 输出的里程计与配准点云输入 **EGO-Planner**，生成无碰撞轨迹。
3. **控制**：将 EGO-Planner 生成的轨迹输入 **MPC 控制器**，输出电机 PWM 指令。

> **注意**：真值话题 `/airsim_node/drone_1/debug/pose_gt` 仅用于本地调试，**正式比赛不可用**。

---

## 2. 话题接口对齐表

| 来源 | 话题 | 消息类型 | 坐标系 / 备注 |
|------|------|----------|---------------|
| **仿真器** | `/airsim_node/drone_1/lidar` | `sensor_msgs/PointCloud2` | NED，传感器局部坐标系。**缺少 `ring` 与 `time` 字段。** |
| **仿真器** | `/airsim_node/drone_1/imu/imu` | `sensor_msgs/Imu` | NED 坐标系。 |
| **LIO-SAM** (输入) | `/points_raw` | `sensor_msgs/PointCloud2` | ENU，LiDAR 局部坐标系。**必须包含 `ring` 与 `time`。** |
| **LIO-SAM** (输入) | `/imu_raw` | `sensor_msgs/Imu` | ENU，符合 REP-105 规范。 |
| **LIO-SAM** (输出) | `/lio_sam/mapping/odometry_incremental` | `nav_msgs/Odometry` | `map` 坐标系，ENU，高频位姿。 |
| **LIO-SAM** (输出) | `/lio_sam/mapping/cloud_registered` | `sensor_msgs/PointCloud2` | `map` 坐标系，ENU，去畸变稠密点云。 |
| **EGO-Planner** (输入) | `cloud_topic` | `sensor_msgs/PointCloud2` | `world` 坐标系，ENU，障碍物地图。 |
| **EGO-Planner** (输入) | `odom_topic` | `nav_msgs/Odometry` | `world` 坐标系，ENU，连续位姿。 |
| **EGO-Planner** (输出) | *(自定义轨迹消息)* | 待定 | B-spline 轨迹（位置、速度、加速度、偏航）。 |
| **MPC** (输入) | *(自定义轨迹消息)* | 待定 | 以 100 Hz 采样的参考轨迹。 |
| **MPC** (输出) | `/airsim_node/drone_1/rotor_pwm_cmd` | `airsim_ros/RotorPWM` | 直接电机控制指令。 |

---

## 3. 管线阶段与适配层

### 3.1 仿真器 → LIO-SAM 适配器 (`sim_lidar_adapter`)

**状态**：❌ **尚不存在。这是当前最关键缺失模块。**

在 LIO-SAM 能初始化之前，该适配节点必须完成以下转换：

1. **坐标系转换 NED → ENU**
   - 对点云与 IMU 线加速度 / 角速度施加静态旋转（等效于 180° roll + 90° yaw）。
2. **注入 `ring` 字段**
   - 计算仰角：`atan2(z, sqrt(x²+y²))`。
   - 将 `[-7°, +52°]` 等分为 32 档（0–31）。
3. **注入 `time` 字段**
   - 计算水平角：`atan2(y, x)`。
   - 按 10 Hz 扫描周期映射到 `[0, 0.1s]`：`time = azimuth / (2π) * 0.1`。
4. **重发布到 `/points_raw`** 与 `/imu_raw`。
5. **时间同步**
   - 以 **IMU 消息时间戳** 作为主时钟参考，而不是 `ros::Time::now()`，避免主机与仿真器时钟偏移。
6. **IMU 外参**
   - 确保 `LIO-SAM-raw/config/params.yaml` 中的 `extrinsicRot` / `extrinsicRPY` 与仿真器 IMU-to-LiDAR 变换一致。

### 3.2 LIO-SAM → EGO-Planner 桥接

**状态**：⚠️ **部分就绪（话题已存在，但重映射与 TF 需验证）。**

- LIO-SAM 在 ENU 下输出里程计与配准点云。
- EGO-Planner 期望输入在 `world` / ENU 坐标系下。
- 可能需要轻量级桥接节点或 TF 发布器，确保 `frame_id` 字符串严格匹配（`map` 与 `world`）。
- **点云密度**：若 `/cloud_registered` 对 EGO-Planner 的 `plan_env` 过于稠密，需插入 VoxelGrid 降采样节点。
- **里程计连续性**：若 LIO-SAM 输出存在间隙，需添加状态预测器或 TF 外推层。

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

## 4. 开发顺序

1. **`sim_lidar_adapter`** — 影响最大。没有它，LIO-SAM 无法初始化。
2. **LIO-SAM 参数调优** — 更新 `params.yaml` 外参与噪声参数以匹配仿真无人机。对照 `debug/pose_gt` 验证。
3. **LIO-SAM → EGO-Planner 对接** — 重映射话题，验证局部地图正常构建，确认规划器能无崩溃输出轨迹。
4. **冻结轨迹消息定义** — 与 MPC  teammate 就 `.msg` 定义达成一致，提交至 `airsim_ros` 或 `quadrotor_msgs`。
5. **`ego_to_mpc_bridge`** — 将 EGO-Planner 轨迹采样为已约定的消息格式。MPC 组可并行使用虚拟轨迹发布器进行开发。
6. **全栈联调** — 在仿真器中运行完整管线，调试时序 / TF / frame_id 问题。
7. **Docker 打包** — 更新 `setup.bash` 自动启动完整管线，构建提交镜像并导出 `test.tar`。

---

## 5. 架构图

```
┌─────────────┐      ┌──────────────────┐      ┌──────────────┐
│  RMUA Sim   │─────▶│ sim_lidar_adapter│─────▶│   LIO-SAM    │
│  (NED)      │      │ (NED→ENU, ring,  │      │   (ENU)      │
│             │      │  time injection) │      │              │
└─────────────┘      └──────────────────┘      └──────┬───────┘
                                                      │
                           ┌──────────────────────────┘
                           ▼
                    ┌──────────────┐
                    │ EGO-Planner  │
                    │ (plan_manage)│
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │ego_to_mpc_   │
                    │    bridge    │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │ MPC Tracker  │
                    │(controller)  │
                    └──────┬───────┘
                           ▼
                    ┌──────────────┐
                    │  Simulator   │
                    │ (RotorPWM)   │
                    └──────────────┘
```

---

## 6. 待你决策的开放问题

以下选择将直接影响实现方式。请在开始编码前确认：

### 决策 A：轨迹消息标准
EGO-Planner 如何向 MPC 发送轨迹？

- **选项 1 — 新建自定义消息**（`Trajectory` / `TrajectoryPoint`），放在 `airsim_ros` 中。干净、专用，但需双方共同约定 `.msg` 文件。**（推荐）**
- **选项 2 — 复用现有消息**。若 `ego-planner-raw` 或 `rpg_quadrotor_common` 中已有合适消息，可直接使用。启动快，但可能有多余字段或缺少必要字段。
- **选项 3 — 使用 `nav_msgs/Path` + 自定义话题约定**。简单，但会丢失速度/加速度/偏航信息，不足以支持 MPC。

> **我的建议**：选项 1。定义一个包含 `time_from_start`、位置、速度、加速度、偏航及偏航角速度的简洁 `TrajectoryPoint[]` 消息。

### 决策 B：EGO-Planner 的局部地图来源
EGO-Planner 消费哪路点云？

- **选项 1 — 直接喂入 LIO-SAM 的 `/cloud_registered`**。最简单。EGO-Planner 的 `plan_env` 会自行构建局部地图。风险：点云可能过密或过疏，取决于 LIO-SAM 调参。**（推荐）**
- **选项 2 — 先对 `/cloud_registered` 做 VoxelGrid 降采样**。增加一个滤波节点，但能保证 `plan_env` 的点密度上限。
- **选项 3 — LiDAR 与双目深度融合**。更鲁棒，但复杂度显著增加，对当前比赛环境可能没有必要。

> **我的建议**：先选选项 1。若 EGO-Planner 的 CPU 负载过高或局部地图表现不佳，再追加 VoxelGrid 节点（选项 2）作为无侵入替换。

### 决策 C：起飞与任务启动逻辑
谁触发初始起飞？规划器何时介入？

- **选项 1 — `setup.bash` 调用一次 `/takeoff` 服务，等待稳定悬停后再启动所有节点**。简单，但所有节点在容器内同时启动，可能错过最初几秒传感器数据。
- **选项 2 — 所有节点立即启动（包括 `sim_lidar_adapter` 与 LIO-SAM）。一个轻量级状态机节点等待 `/airsim_node/initial_pose`，调用 takeoff，随后通知 EGO-Planner 开始规划**。更鲁棒，但需编写一个小型任务管理节点。**（推荐）**

> **我的建议**：选项 2。`imu_gps_odometry` 节点已具备阻塞等待 `/airsim_node/initial_pose` 的模式；将其扩展为微型任务管理器是自然的做法，可避免竞态条件。

### 决策 D：LIO-SAM 失效 / 漂移时的降级策略
若 LIO-SAM 丢跟踪或漂移，如何处理？

- **选项 1 — 无降级策略。LIO-SAM 失效则任务结束**。代码路径最简单。若比赛允许多次运行或环境对 LIO-SAM 友好，则可接受。**（推荐）**
- **选项 2 — 降级到 ESKF 里程计 (`/eskf_odom`)**。现有 `imu_gps_odometry` 包已发布该话题。若 LIO-SAM 里程计停止更新，EGO-Planner 可切换至该来源。增加复杂度，但提高鲁棒性。

> **我的建议**：第一里程碑选选项 1。仅在联调中证明 LIO-SAM 在特定仿真环境中存在可靠性问题时，再引入选项 2。

---

## 7. 已知陷阱

1. **硬编码路径**：`controllerTest.cpp` 中包含绝对轨迹文件路径。在 Docker 提交前必须改为参数化加载。
2. **时钟偏移**：不要依赖 `ros::Time::now()`。以 IMU 消息时间戳作为主时钟。
3. **Frame ID 不匹配**：TF 或消息 `frame_id` 中 `map`、`world`、`odom` 的不一致将静默破坏 LIO-SAM 或 EGO-Planner。
4. **提交镜像禁止 GUI**：最终 Docker 镜像不能启动 RViz 或任何 X11 程序。
5. **EGO-Planner 需要初始目标**：确保任务管理器或启动文件在首个航点附近提供默认目标，或订阅 `/airsim_node/end_goal`。

---

*最后更新：2026-05-23*
