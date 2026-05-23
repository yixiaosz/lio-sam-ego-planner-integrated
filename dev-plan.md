# RMUA2026 Integration Development Plan

> This document replaces `project-description-short.md`. It is the living integration plan for the Simulator → LIO-SAM → EGO-Planner → MPC pipeline.

---

## 1. Goal

Build a full autonomous navigation pipeline for the RMUA 2026 AirSim-based quadrotor simulator:

1. **Perception / SLAM**: Feed simulator LiDAR + IMU into **LIO-SAM** to produce real-time mapping and localization.
2. **Planning**: Feed LIO-SAM odometry and registered point clouds into **EGO-Planner** to generate collision-free trajectories.
3. **Control**: Feed EGO-Planner trajectories into the **MPC controller** to output rotor PWM commands.

> **Note**: The ground-truth topic `/airsim_node/drone_1/debug/pose_gt` is for local debugging only and will **not** be available in the real competition.

---

## 2. Topic Interface Alignment

| Source | Topic | Message Type | Frame / Notes |
|--------|-------|--------------|---------------|
| **Simulator** | `/airsim_node/drone_1/lidar` | `sensor_msgs/PointCloud2` | NED, sensor-local frame. **Missing `ring` and `time` fields.** |
| **Simulator** | `/airsim_node/drone_1/imu/imu` | `sensor_msgs/Imu` | NED frame. |
| **LIO-SAM** (input) | `/points_raw` | `sensor_msgs/PointCloud2` | ENU, lidar-local frame. **Must contain `ring` and `time`.** |
| **LIO-SAM** (input) | `/imu_raw` | `sensor_msgs/Imu` | ENU, REP-105 compliant. |
| **LIO-SAM** (output) | `/lio_sam/mapping/odometry_incremental` | `nav_msgs/Odometry` | `map` frame, ENU, high-rate pose. |
| **LIO-SAM** (output) | `/lio_sam/mapping/cloud_registered` | `sensor_msgs/PointCloud2` | `map` frame, ENU, deskewed dense cloud. |
| **EGO-Planner** (input) | `cloud_topic` | `sensor_msgs/PointCloud2` | `world` frame, ENU, obstacle map. |
| **EGO-Planner** (input) | `odom_topic` | `nav_msgs/Odometry` | `world` frame, ENU, continuous pose. |
| **EGO-Planner** (output) | *(custom trajectory msg)* | TBD | B-spline trajectory (position, velocity, acceleration, yaw). |
| **MPC** (input) | *(custom trajectory msg)* | TBD | Sampled reference trajectory at 100 Hz. |
| **MPC** (output) | `/airsim_node/drone_1/rotor_pwm_cmd` | `airsim_ros/RotorPWM` | Direct motor commands. |

---

## 3. Pipeline Stages & Adapter Layers

### 3.1 Simulator → LIO-SAM Adapter (`sim_lidar_adapter`)

**Status**: ❌ **Does not exist yet. This is the primary missing piece.**

The adapter node must perform the following transformations before LIO-SAM can initialize:

1. **Frame conversion NED → ENU**
   - Apply a static rotation (180° roll + 90° yaw equivalent) to both point clouds and IMU linear acceleration / angular velocity.
2. **Inject `ring` field**
   - Compute elevation: `atan2(z, sqrt(x²+y²))`.
   - Map `[-7°, +52°]` into 32 equal bins (0–31).
3. **Inject `time` field**
   - Compute azimuth: `atan2(y, x)`.
   - Map to `[0, 0.1s]` for a 10 Hz sweep: `time = azimuth / (2π) * 0.1`.
4. **Republish to `/points_raw`** and `/imu_raw`.
5. **Time synchronization**
   - Use the **IMU message timestamp** as the master clock reference rather than `ros::Time::now()`, to avoid host/simulator clock skew.
6. **IMU extrinsics**
   - Ensure `LIO-SAM-raw/config/params.yaml` has `extrinsicRot` / `extrinsicRPY` matching the simulated IMU-to-LiDAR transform.

### 3.2 LIO-SAM → EGO-Planner Bridge

**Status**: ⚠️ **Partially ready (topics exist, but remapping + TF need verification).**

- LIO-SAM outputs odometry and registered clouds in ENU.
- EGO-Planner expects inputs in a `world` / ENU frame.
- A lightweight bridge or TF publisher may be needed to ensure `frame_id` strings match exactly (`map` vs `world`).
- **Point cloud density**: If `/cloud_registered` is too heavy for EGO-Planner's `plan_env`, insert a VoxelGrid downsampling node.
- **Odometry continuity**: If LIO-SAM output is bursty, add a state-predictor or TF extrapolation layer.

### 3.3 EGO-Planner → MPC Interface (`ego_to_mpc_bridge`)

**Status**: ❌ **Not defined yet. Requires team agreement on message format.**

EGO-Planner generates a B-spline trajectory (position, velocity, acceleration, yaw, yaw rate). The MPC team needs a stable reference stream.

**Proposed message interface** (to be added to `airsim_ros` or `quadrotor_msgs`):

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

**Bridge node responsibilities:**
1. Subscribe to EGO-Planner trajectory output.
2. Buffer the latest valid trajectory.
3. Sample the trajectory at the current time to produce a reference state.
4. Publish at the MPC control rate (100 Hz).
5. Handle edge cases: no trajectory yet (publish hover setpoint), trajectory expired (request replan).

### 3.4 MPC → Simulator

**Status**: ✅ **Partially implemented in `controller_test`.**

- The existing `controllerTest.cpp` loads waypoints from a text file and calls `rpg_mpc::MpcController::run()`.
- Once the trajectory message is defined, replace the file loader with a subscriber.
- Keep the existing `RotorPWM` publisher and motor index mapping (`0:RF, 1:LR, 2:LF, 3:RR`).

---

## 4. Development Order

1. **`sim_lidar_adapter`** — Highest impact. Without it, LIO-SAM cannot initialize.
2. **LIO-SAM tuning** — Update `params.yaml` extrinsics and noise values for the simulated drone. Verify against `debug/pose_gt`.
3. **LIO-SAM → EGO-Planner hookup** — Remap topics, verify local map builds, confirm planner outputs trajectories without crashing.
4. **Freeze Trajectory MSG** — Agree with the MPC teammate on the exact `.msg` definition. Check it into `airsim_ros` or `quadrotor_msgs`.
5. **`ego_to_mpc_bridge`** — Sample EGO-Planner trajectories into the agreed message. The MPC team can develop against a dummy trajectory publisher in parallel.
6. **Full stack integration test** — Run everything together in the simulator and debug timing / TF / frame_id issues.
7. **Dockerize** — Update `setup.bash` to auto-launch the full pipeline, build the submission image, and export `test.tar`.

---

## 5. Architecture Diagram

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

## 6. Open Decisions Requiring Your Input

The following choices materially affect the implementation. Please decide before we start coding:

### Decision A: Trajectory Message Standard
How should EGO-Planner talk to MPC?

- **Option 1 — Create a new custom message** (`Trajectory` / `TrajectoryPoint`) inside `airsim_ros`. Clean and purpose-built, but requires both sides to agree on the `.msg` file. *(Recommended)*
- **Option 2 — Re-use an existing message** from `ego-planner-raw` or `rpg_quadrotor_common` if one already exists. Faster to start, but may carry fields we don't need or lack fields we do.
- **Option 3 — Use `nav_msgs/Path` + custom topic conventions**. Simple, but loses velocity/acceleration/yaw information. Not sufficient for MPC.

> **My recommendation**: Option 1. Define a simple `TrajectoryPoint[]` message with `time_from_start`, position, velocity, acceleration, yaw, and yaw_dot.

### Decision B: Local Map Source for EGO-Planner
What point cloud does EGO-Planner consume?

- **Option 1 — Feed LIO-SAM `/cloud_registered` directly**. Simplest. EGO-Planner's `plan_env` will build its own local map from it. Risk: the cloud may be too dense or too sparse depending on LIO-SAM tuning.
- **Option 2 — Feed a VoxelGrid-downsampled version of `/cloud_registered`**. Adds a filtering node but guarantees a bounded point density for `plan_env`.
- **Option 3 — Fuse LiDAR with stereo depth**. More robust, but significantly more complex and probably unnecessary for the competition environment.

> **My recommendation**: Start with Option 1. If EGO-Planner's CPU load is too high or its local map behaves poorly, add a VoxelGrid node (Option 2) as a drop-in replacement without changing the rest of the pipeline.

### Decision C: Takeoff & Mission Start Logic
Who triggers the initial takeoff and when does the planner activate?

- **Option 1 — `setup.bash` calls `/takeoff` service once, waits for a stable hover, then launches all nodes**. Simple, but all nodes start inside the container at once and may miss the first few seconds of sensor data.
- **Option 2 — All nodes start immediately (including `sim_lidar_adapter` and LIO-SAM). A small state-machine node waits for `/airsim_node/initial_pose`, calls takeoff, then signals EGO-Planner to start planning.** More robust, but requires writing a lightweight mission manager node.

> **My recommendation**: Option 2. The `imu_gps_odometry` node already blocks on `/airsim_node/initial_pose`; extending this pattern into a tiny mission manager is clean and avoids race conditions.

### Decision D: Failover / Degraded Mode
What happens if LIO-SAM loses tracking or drifts?

- **Option 1 — No failover. If LIO-SAM dies, the run fails.** Simplest code path. Acceptable if the competition allows multiple runs or if the environment is LIO-SAM-friendly.
- **Option 2 — Fallback to ESKF odometry (`/eskf_odom`)**. The existing `imu_gps_odometry` package publishes this. EGO-Planner could switch to it if LIO-SAM odometry stops updating. Adds complexity but increases robustness.

> **My recommendation**: Option 1 for the first milestone. Add Option 2 only if integration tests show LIO-SAM reliability issues in the specific simulator environments.

---

## 7. Known Pitfalls

1. **Hardcoded paths**: `controllerTest.cpp` contains an absolute trajectory file path. Must be parameterized before Docker submission.
2. **Clock skew**: Do not rely on `ros::Time::now()`. Use IMU message timestamps as the master clock.
3. **Frame IDs**: A mismatch between `map`, `world`, and `odom` in TF or message `frame_id`s will silently break LIO-SAM or EGO-Planner.
4. **No GUI in submission**: The final Docker image cannot launch RViz or any X11 application.
5. **EGO-Planner expects an initial goal**: Ensure the mission manager or launch file provides a default goal near the first waypoint, or subscribe to `/airsim_node/end_goal`.

---

*Last updated: 2026-05-23*
