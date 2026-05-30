# RMUA2026 Integration Development Plan (FAST-LIVO2 Edition)

> This document replaces `dev-plan.md.bak`. It is the living integration plan for the Simulator → FAST-LIVO2 → EGO-Planner → MPC pipeline.

---

## 1. Goal

Build a full autonomous navigation pipeline for the RMUA 2026 AirSim-based quadrotor simulator:

1. **Perception / SLAM**: Feed simulator LiDAR + IMU + front-left camera into **FAST-LIVO2** (Full LIVO mode) to produce real-time mapping and localization.
2. **Planning**: Feed FAST-LIVO2 odometry and registered point clouds into **EGO-Planner** to generate collision-free trajectories.
3. **Control**: Feed EGO-Planner trajectories into the **MPC controller** to output rotor PWM commands.

> **Note**: The ground-truth topic `/airsim_node/drone_1/debug/pose_gt` is for local debugging only and will **not** be available in the real competition.

---

## 2. Topic Interface Alignment

| Source | Topic | Message Type | Frame / Notes |
|--------|-------|--------------|---------------|
| **Simulator** | `/airsim_node/drone_1/lidar` | `sensor_msgs/PointCloud2` | NED, sensor-local frame. Standard PointCloud2 (no ring/time). |
| **Simulator** | `/airsim_node/drone_1/imu/imu` | `sensor_msgs/Imu` | NED frame. |
| **Simulator** | `/airsim_node/drone_1/front_left/Scene` | `sensor_msgs/Image` | NED, front-left camera. Used as monocular input for FAST-LIVO2 LIVO mode. |
| **Adapter** (input) | — | — | Receives simulator topics above. |
| **Adapter** (output) | `/sim/lidar` | `sensor_msgs/PointCloud2` | ENU, lidar-local frame. Standard PointCloud2, no ring/time injection required. |
| **Adapter** (output) | `/sim/imu` | `sensor_msgs/Imu` | ENU, REP-105 compliant. |
| **Adapter** (output) | `/sim/camera` | `sensor_msgs/Image` | ENU-aligned image stream for FAST-LIVO2 visual thread. |
| **FAST-LIVO2** (input) | `lid_topic` | `sensor_msgs/PointCloud2` | Configurable. Standard PointCloud2 with `lidar_type: 3`. |
| **FAST-LIVO2** (input) | `imu_topic` | `sensor_msgs/Imu` | Configurable. |
| **FAST-LIVO2** (input) | `img_topic` | `sensor_msgs/Image` | Configurable. Monocular front-left image. |
| **FAST-LIVO2** (output) | `/aft_mapped_to_init` | `nav_msgs/Odometry` | `camera_init` frame, ENU, EKF-fused pose at LiDAR rate. |
| **FAST-LIVO2** (output) | `/imu_prop_odom` | `nav_msgs/Odometry` | `camera_init` frame, ENU, high-rate IMU-propagated pose. |
| **FAST-LIVO2** (output) | `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` frame, ENU, deskewed dense cloud. |
| **FAST-LIVO2** (output) | `/path` | `nav_msgs/Path` | Trajectory history. |
| **FAST-LIVO2** (output) | `/rgb_img` | `sensor_msgs/Image` | Republished image with visual tracking overlay (when LIVO active). |
| **Bridge** (output) | `odom_topic` | `nav_msgs/Odometry` | `world` frame, ENU, continuous pose. Renamed from `/aft_mapped_to_init`. |
| **Bridge** (output) | `cloud_topic` | `sensor_msgs/PointCloud2` | `world` frame, ENU, obstacle map. Relayed from `/cloud_registered`. |
| **EGO-Planner** (output) | *(custom trajectory msg)* | TBD | B-spline trajectory (position, velocity, acceleration, yaw). |
| **MPC** (input) | *(custom trajectory msg)* | TBD | Sampled reference trajectory at 100 Hz. |
| **MPC** (output) | `/airsim_node/drone_1/rotor_pwm_cmd` | `airsim_ros/RotorPWM` | Direct motor commands. |

---

## 3. Pipeline Stages & Adapter Layers

### 3.1 Simulator → FAST-LIVO2 Adapter (`sim_sensor_adapter`)

**Status**: ❌ **Does not exist yet. This is the primary missing piece.**

The adapter node must perform the following transformations before FAST-LIVO2 can initialize:

1. **Frame conversion NED → ENU**
   - Apply a static rotation to point clouds, IMU linear acceleration / angular velocity, and image header frame_ids.
   - For the image stream, the pixel content does not change; only the `frame_id` in the message header is rewritten to indicate ENU alignment.
2. **Republish LiDAR**
   - Subscribe to `/airsim_node/drone_1/lidar`.
   - Transform point coordinates from NED to ENU.
   - Publish as standard `sensor_msgs/PointCloud2` to `/sim/lidar`.
   - **No ring or time injection is required** because FAST-LIVO2 will run with `preprocess.lidar_type: 3` (standard PointCloud2 handler).
3. **Republish IMU**
   - Subscribe to `/airsim_node/drone_1/imu/imu`.
   - Rotate linear acceleration and angular velocity from NED to ENU.
   - Publish to `/sim/imu`.
4. **Republish Camera**
   - Subscribe to `/airsim_node/drone_1/front_left/Scene`.
   - Rewrite header `frame_id` to ENU-aligned camera frame.
   - Publish to `/sim/camera`.
5. **Time synchronization**
   - Use the **IMU message timestamp** as the master clock reference rather than `ros::Time::now()`, to avoid host/simulator clock skew.
6. **Camera-LiDAR-IMU extrinsics**
   - The simulator provides deterministic sensor poses. The adapter does not need to estimate extrinsics, but the FAST-LIVO2 YAML must contain:
     - `extrinsic_T` / `extrinsic_R`: LiDAR-to-IMU transform
     - `Rcl` / `Pcl`: Camera-to-LiDAR rotation and translation
     - Camera intrinsics: `fx`, `fy`, `cx`, `cy`, distortion model parameters

### 3.2 FAST-LIVO2 → EGO-Planner Bridge (`fastlivo2_to_ego_bridge`)

**Status**: ❌ **Does not exist yet.**

FAST-LIVO2 outputs odometry and registered clouds in the `camera_init` frame. EGO-Planner expects inputs in a `world` / ENU frame.

**Bridge node responsibilities:**
1. Subscribe to `/aft_mapped_to_init` and `/cloud_registered`.
2. Rename `frame_id` from `camera_init` to `world` in both messages.
3. Publish odometry to EGO-Planner's configured `odom_topic`.
4. Publish point cloud to EGO-Planner's configured `cloud_topic`.
5. Optionally subscribe to `/imu_prop_odom` and use it as a high-rate fallback or smoothing input if EGO-Planner's internal state predictor needs it.
6. **TF broadcast**: Ensure `world` → `base_link` TF is published consistently, either by republishing FAST-LIVO2's TF under new frame names or by letting EGO-Planner consume odometry directly.

**Point cloud density**: If `/cloud_registered` is too heavy for EGO-Planner's `plan_env`, insert a VoxelGrid downsampling node between the bridge and EGO-Planner.

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

## 4. Legacy Stereo VO Package (`odometry`)

**Status**: ⚠️ **Sidelined. Retained in workspace but not in competition launch path.**

The existing `odometry` package performs stereo visual odometry using front-left + front-right cameras, optical flow, and g2o. It was architected as a debug/tutorial node:

- It subscribes to `/airsim_node/drone_1/pose` (simulator ground-truth/local pose), making it non-functional for competition.
- It publishes only debug outputs (`/display_image`, `/cur_frame_pcl`, `/local_map_pcl`) and no odometry message.
- It **cannot feed into FAST-LIVO2**, because FAST-LIVO2's LIVO mode consumes raw monocular images directly and performs its own direct photometric alignment internally.

**Dispositions:**
- **Source code**: Keep in `src/odometry/` for reference and local debug comparisons.
- **Build**: Remove from `catkin_make` whitelist and `setup.bash` auto-launch for the competition image.
- **Runtime**: Do not launch in the competition pipeline. FAST-LIVO2 subscribes to `front_left/Scene` directly.

---

## 5. FAST-LIVO2 Configuration Requirements

A new configuration YAML must be created for the simulator (e.g., `fast_livo2_sim.yaml`). Key parameters:

**Common:**
- `lid_topic`: `/sim/lidar`
- `imu_topic`: `/sim/imu`
- `img_topic`: `/sim/camera`
- `img_en`: `1` (enable visual-inertial thread)
- `lidar_en`: `1`
- `slam_mode`: `2` (LIVO)

**Preprocess:**
- `lidar_type`: `3` (standard PointCloud2 / Ouster-style handler)
- `point_filter_num`: `1`
- `blind`: `0.5`

**Extrinsics:**
- `extrinsic_T` / `extrinsic_R`: LiDAR-to-IMU (simulator-determined)
- `Rcl` / `Pcl`: Camera-to-LiDAR rotation and translation (simulator-determined)

**Camera:**
- Camera model type and intrinsics must be populated for Vikit (Pinhole + Radtan expected for AirSim Scene camera).
- Distortion coefficients must be provided if non-zero.

**Publish:**
- `scan_publish_en`: `true`
- `dense_publish_en`: `true`
- `path_en`: `true` (optional, for debug)

---

## 6. Dependency & Build System Changes

### New Dependencies to Add
1. **FAST-LIVO2** cloned into `RMUA2026-dev/basic_dev/src/`
2. **Vikit** (`xuankuzcr/rpg_vikit`) cloned into `src/` — must be built as a catkin package
3. **Sophus** (system-wide install from commit `a621ff`)

### Dependencies to Remove (if LIO-SAM is fully dropped)
- `libgtsam-dev`
- `libgtsam-unstable-dev`
- `LIO-SAM-raw/` reference package (keep directory as upstream reference, but do not build in the competition workspace)

### Build Order
```bash
catkin_make --only-pkg-with-deps fast_livo
# or full workspace build
```

---

## 7. Development Order

1. **`sim_sensor_adapter`** — Highest impact. NED→ENU conversion for LiDAR, IMU, and image. No ring/time injection needed.
2. **FAST-LIVO2 tuning** — Create `fast_livo2_sim.yaml`. Populate camera intrinsics and simulator extrinsics. Verify with `debug/pose_gt`. Tune `acc_cov`, `gyr_cov`, `blind`, and `point_filter_num`.
3. **`fastlivo2_to_ego_bridge`** — Frame rename (`camera_init` → `world`), topic relay, optional VoxelGrid filter.
4. **FAST-LIVO2 → EGO-Planner hookup** — Remap topics, verify local map builds, confirm planner outputs trajectories without crashing.
5. **Freeze Trajectory MSG** — Agree with the MPC teammate on the exact `.msg` definition. Check it into `airsim_ros` or `quadrotor_msgs`.
6. **`ego_to_mpc_bridge`** — Sample EGO-Planner trajectories into the agreed message. The MPC team can develop against a dummy trajectory publisher in parallel.
7. **Full stack integration test** — Run everything together in the simulator and debug timing / TF / frame_id issues.
8. **Dockerize** — Update `setup.bash` to auto-launch the full pipeline, build the submission image, and export `test.tar`.

---

## 8. Architecture Diagram

```
┌─────────────┐      ┌──────────────────┐      ┌──────────────┐
│  RMUA Sim   │─────▶│ sim_sensor_adapter│─────▶│  FAST-LIVO2  │
│  (NED)      │      │ (NED→ENU, no     │      │   (ENU)      │
│             │      │  ring/time inj)  │      │  LIVO mode   │
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

[Sidelined] odometry (stereo VO debug node, not launched)
```

---

## 9. Open Decisions Requiring Your Input

The following choices materially affect the implementation. Please decide before we start coding:

### Decision A: Trajectory Message Standard
How should EGO-Planner talk to MPC?

- **Option 1 — Create a new custom message** (`Trajectory` / `TrajectoryPoint`) inside `airsim_ros`. Clean and purpose-built, but requires both sides to agree on the `.msg` file. *(Recommended)*
- **Option 2 — Re-use an existing message** from `ego-planner-raw` or `rpg_quadrotor_common` if one already exists. Faster to start, but may carry fields we don't need or lack fields we do.
- **Option 3 — Use `nav_msgs/Path` + custom topic conventions**. Simple, but loses velocity/acceleration/yaw information. Not sufficient for MPC.

> **My recommendation**: Option 1. Define a simple `TrajectoryPoint[]` message with `time_from_start`, position, velocity, acceleration, yaw, and yaw_dot.

### Decision B: Local Map Source for EGO-Planner
What point cloud does EGO-Planner consume?

- **Option 1 — Feed FAST-LIVO2 `/cloud_registered` directly**. Simplest. EGO-Planner's `plan_env` will build its own local map from it. Risk: the cloud may be too dense or too sparse depending on tuning. *(Recommended)*
- **Option 2 — Feed a VoxelGrid-downsampled version of `/cloud_registered`**. Adds a filtering node but guarantees a bounded point density for `plan_env`.
- **Option 3 — Fuse LiDAR with stereo depth**. More robust, but significantly more complex and probably unnecessary for the competition environment.

> **My recommendation**: Start with Option 1. If EGO-Planner's CPU load is too high or its local map behaves poorly, add a VoxelGrid node (Option 2) as a drop-in replacement without changing the rest of the pipeline.

### Decision C: Takeoff & Mission Start Logic
Who triggers the initial takeoff and when does the planner activate?

- **Option 1 — `setup.bash` calls `/takeoff` service once, waits for a stable hover, then launches all nodes**. Simple, but all nodes start inside the container at once and may miss the first few seconds of sensor data.
- **Option 2 — All nodes start immediately (including `sim_sensor_adapter` and FAST-LIVO2). A small state-machine node waits for `/airsim_node/initial_pose`, calls takeoff, then signals EGO-Planner to start planning.** More robust, but requires writing a lightweight mission manager node. *(Recommended)*

> **My recommendation**: Option 2. The `imu_gps_odometry` node already blocks on `/airsim_node/initial_pose`; extending this pattern into a tiny mission manager is clean and avoids race conditions.

### Decision D: Failover / Degraded Mode
What happens if FAST-LIVO2 loses tracking or drifts?

- **Option 1 — No failover. If FAST-LIVO2 dies, the run fails.** Simplest code path. Acceptable if the competition allows multiple runs or if the environment is SLAM-friendly. *(Recommended for first milestone)*
- **Option 2 — Fallback to ESKF odometry (`/eskf_odom`)**. The existing `imu_gps_odometry` package publishes this. EGO-Planner could switch to it if FAST-LIVO2 odometry stops updating. Adds complexity but increases robustness.

> **My recommendation**: Option 1 for the first milestone. Add Option 2 only if integration tests show reliability issues in the specific simulator environments.

---

## 10. Known Pitfalls

1. **Hardcoded paths**: `controllerTest.cpp` contains an absolute trajectory file path. Must be parameterized before Docker submission.
2. **Clock skew**: Do not rely on `ros::Time::now()`. Use IMU message timestamps as the master clock.
3. **Frame IDs**: A mismatch between `camera_init`, `world`, and `odom` in TF or message `frame_id`s will silently break FAST-LIVO2 or EGO-Planner.
4. **No GUI in submission**: The final Docker image cannot launch RViz or any X11 application. PCD/TUM saving can be enabled for post-run analysis on the competition server if disk space permits.
5. **EGO-Planner expects an initial goal**: Ensure the mission manager or launch file provides a default goal near the first waypoint, or subscribe to `/airsim_node/end_goal`.
6. **Sophus version lock**: FAST-LIVO2 + Vikit require Sophus commit `a621ff`. Newer Sophus versions break compilation.
7. **Camera extrinsics accuracy**: LIVO mode is sensitive to LiDAR-camera extrinsics (`Rcl` / `Pcl`). Simulator extrinsics are deterministic, but any manual transcription error will cause visual-LiDAR misalignment and drift.
8. **Standard PointCloud2 mode trade-off**: Using `lidar_type: 3` loses Livox-native `offset_time` per-point metadata. If simulator LiDAR closely mimics MID360 scan pattern and drift is observed, consider building a `PointCloud2 → CustomMsg` converter to use `lidar_type: 1`.

---

*Last updated: 2026-05-30*
