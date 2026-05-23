# AGENTS.md

This file contains project-specific information intended for AI coding agents.
Read this first before making any changes to the codebase.

---

## Project Overview

This repository is an integration project for the **RoboMaster University AI Championship (RMUA) 2026 Autonomous Drone Racing** competition.

The high-level goal is to build a full autonomous navigation pipeline for a simulated quadrotor:

1. **Perception / SLAM**: Feed LiDAR and IMU data from the RMUA AirSim-based simulator into **LIO-SAM** to produce real-time mapping and localization.
2. **Planning**: Feed LIO-SAM odometry and registered point clouds into **EGO-Planner** to generate collision-free, dynamically feasible trajectories.
3. **Control**: Feed the planned trajectory into an **MPC controller** that outputs rotor PWM commands to fly the drone.

The repository contains three top-level directories:

- `LIO-SAM-raw/` — Unmodified reference codebase of LIO-SAM (lidar-inertial odometry).
- `ego-planner-raw/` — Unmodified reference codebase of EGO-Planner (local trajectory planner for quadrotors).
- `RMUA2026-dev/` — **Work-in-progress integration codebase** where active development happens. This is a ROS Noetic catkin workspace that interfaces with the competition simulator and contains custom nodes for odometry fusion, visual odometry, and MPC control.

> **Important**: The `RMUA2026-dev/` directory is the only actively developed part. The `-raw/` directories are frozen upstream references. If you need to modify LIO-SAM or EGO-Planner behavior for the simulator, create adapted copies inside `RMUA2026-dev/` rather than editing the raw trees directly.

---

## Technology Stack

| Layer | Technology |
|-------|------------|
| OS | Ubuntu 20.04 |
| ROS Distro | ROS Noetic (desktop-full) |
| Build System | CMake + catkin_make |
| Container | Docker ( Nvidia-Docker for GPU support ) |
| C++ Standards | C++11 (LIO-SAM), C++17 (RMUA controller / MPC) |
| Key Libraries | PCL, OpenCV, Eigen3, GTSAM, Boost, g2o, ACADO (codegen), qpOASES |

### External Dependencies

- **ROS Noetic**: `ros-noetic-desktop-full`, `python3-catkin-tools`
- **LIO-SAM extras**: `libgtsam-dev`, `libgtsam-unstable-dev`
- **EGO-Planner extras**: `libarmadillo-dev`
- **RMUA dev extras**: `ros-noetic-geographic-msgs`, `ros-noetic-tf2-sensor-msgs`, `ros-noetic-tf2-geometry-msgs`, `ros-noetic-image-transport`, `net-tools`

---

## Repository Structure

```
.
├── LIO-SAM-raw/               # Reference: LIO-SAM ROS package
│   ├── src/                   #   imageProjection, featureExtraction, mapOptmization, imuPreintegration
│   ├── config/params.yaml     #   Main parameter file (topics, extrinsics, sensor config)
│   ├── launch/run.launch      #   Standard launch file
│   └── msg/ srv/ include/
├── ego-planner-raw/           # Reference: EGO-Planner catkin workspace
│   ├── src/planner/           #   plan_manage, plan_env, path_searching, bspline_opt, traj_utils
│   └── src/uav_simulator/     #   Simulator utilities: so3_control, map_generator, mockamap, waypoint_generator
├── RMUA2026-dev/              # Active development workspace
│   ├── basic_dev/             #   Catkin workspace root
│   │   ├── src/
│   │   │   ├── basic_dev/             # Example node: AirSim ROS interface demo
│   │   │   ├── airsim_ros/            # Custom AirSim ROS messages (VelCmd, RotorPWM, Takeoff, etc.)
│   │   │   ├── imu_gps_odometry/      # ESKF-based IMU+GPS odometry fusion → publishes /eskf_odom
│   │   │   ├── odometry/              # Stereo visual odometry using g2o + optical flow
│   │   │   ├── controller/            # MPC controller test node + PD controller
│   │   │   ├── rpg_mpc/               # MPC library (ACADO-generated solver + qpOASES + Eigen wrapper)
│   │   │   ├── rpg_quadrotor_common/  # Common quadrotor types: state estimates, trajectories, control commands
│   │   │   ├── eigen_catkin/          # Eigen catkin wrapper
│   │   │   └── catkin_simple/         # Simplified catkin CMake macros
│   │   ├── Dockerfile               # Competition submission image build
│   │   ├── setup.bash               # Container entrypoint: launches basic_dev, imu_gps_odometry, controller_test
│   │   └── run_basic_dev.sh         # Host script to run the Docker container
│   ├── manual_compile.md          # Host-side build & run instructions (no Docker)
│   ├── README.md                  # Chinese-language tutorial for simulator setup and Docker workflow
│   ├── RMUA-codebase-doc.md       # Chinese-language doc: simulator ROS topics, build commands, drone params
│   └── RMUA-simulator-doc.md      # Chinese-language doc: official simulator usage instructions
├── project-description-short.md   # English integration overview and topic alignment table
├── RMUA-codebase-doc.md           # (duplicate at root)
├── RMUA-simulator-doc.md          # (duplicate at root)
└── AGENTS.md                      # This file
```

---

## Build System & Commands

### Standard Host Build (RMUA2026-dev)

```bash
cd RMUA2026-dev/basic_dev/
source /opt/ros/noetic/setup.bash

# Full workspace build (after cleaning old artifacts from another machine)
rm -rf build devel
catkin_make

# Or build specific packages only
catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros;basic_dev"
catkin_make --only-pkg-with-deps airsim_ros imu_gps_odometry controller_test

# Source the result
source devel/setup.bash
```

### Docker Build (Competition Submission Format)

```bash
cd RMUA2026-dev/basic_dev/
docker build -t basic_dev .
docker image save basic_dev > test.tar   # This .tar is the submission artifact
```

The `Dockerfile`:
- Base image: `osrf/ros:noetic-desktop-full-focal`
- Installs extra ROS packages and tools.
- Copies `src/` into `/basic_dev/src/`.
- Runs `catkin_make` at build time.
- Entrypoint is `setup.bash`, which auto-starts the ROS nodes.

### Build References (Raw Codebases)

**LIO-SAM** (inside `LIO-SAM-raw/`):
```bash
catkin_make   # or place inside a catkin workspace src/ folder
```

**EGO-Planner** (inside `ego-planner-raw/`):
```bash
sudo apt install libarmadillo-dev
catkin_make -DCMAKE_BUILD_TYPE=Release
```

---

## ROS Architecture & Topic Interface

### Simulator Outputs (AirSim / RMUA Simulator)

The simulator publishes the following topics. Nodes in `RMUA2026-dev` subscribe to these:

| Topic | Type | Purpose |
|-------|------|---------|
| `/airsim_node/drone_1/lidar` | `sensor_msgs/PointCloud2` | LiDAR point cloud (NED frame, needs ring/time injection for LIO-SAM) |
| `/airsim_node/drone_1/imu/imu` | `sensor_msgs/Imu` | IMU data (NED frame) |
| `/airsim_node/drone_1/gps` | `geometry_msgs/PoseStamped` | GPS pose with noisy attitude |
| `/airsim_node/drone_1/front_left/Scene` | `sensor_msgs/Image` | Front-left camera |
| `/airsim_node/drone_1/front_right/Scene` | `sensor_msgs/Image` | Front-right camera |
| `/airsim_node/drone_1/debug/pose_gt` | `geometry_msgs/PoseStamped` | **Ground-truth pose (debug only, NOT available in real competition)** |
| `/airsim_node/initial_pose` | `geometry_msgs/PoseStamped` | Start pose |
| `/airsim_node/end_goal` | `geometry_msgs/PoseStamped` | Goal position |

### Simulator Inputs (Control Commands)

| Topic | Type | Purpose |
|-------|------|---------|
| `/airsim_node/drone_1/rotor_pwm_cmd` | `airsim_ros/RotorPWM` | Direct motor PWM (0: RF, 1: LR, 2: LF, 3: RR) |
| `/airsim_node/drone_1/vel_cmd_body_frame` | `airsim_ros/VelCmd` | Velocity command in body frame |

### Simulator Services

| Service | Type | Purpose |
|---------|------|---------|
| `/airsim_node/drone_1/takeoff` | `airsim_ros/Takeoff` | Takeoff |
| `/airsim_node/drone_1/land` | `airsim_ros/Land` | Land |
| `/airsim_node/reset` | `airsim_ros/Reset` | Reset simulation |

### Internal Pipeline Topics

| Topic | Producer | Consumer | Notes |
|-------|----------|----------|-------|
| `/eskf_odom` | `imu_gps_odometry` | `controller_test` | ESKF-fused odometry (nav_msgs/Odometry) |
| `/display_image` | `odometry` | (RViz/debug) | Visual odometry debug image |
| `/cur_frame_pcl`, `/local_map_pcl` | `odometry` | (RViz/debug) | VO local map point clouds |

### LIO-SAM ↔ EGO-Planner Alignment (Planned Integration)

Per `project-description-short.md`, the intended adapter layer must perform:

- **Frame conversion**: AirSim outputs NED; LIO-SAM requires ENU (ROS REP-105).
- **Point cloud field injection**: AirSim LiDAR lacks `ring` and `time` fields. LIO-SAM's `imageProjection.cpp` requires both.
  - `ring`: computed from elevation angle `atan2(z, sqrt(x²+y²))`, mapped into 32 bins across `[-7°, +52°]`.
  - `time`: computed from azimuth angle `atan2(y, x)`, mapped to `[0, 0.1s]` for a 10Hz sweep.
- **LIO-SAM outputs**:
  - `/lio_sam/mapping/odometry_incremental` (`nav_msgs/Odometry`)
  - `/lio_sam/mapping/cloud_registered` (`sensor_msgs/PointCloud2`)
- **EGO-Planner inputs**:
  - `odom_topic`: `nav_msgs/Odometry` in world/ENU frame.
  - `cloud_topic`: `sensor_msgs/PointCloud2` in world/ENU frame.

> As of the current commit, this adapter node does **not** yet exist in the repository. It is the primary missing integration piece.

---

## Code Organization & Module Divisions

### RMUA2026-dev / `basic_dev/src/`

#### `basic_dev`
- **Role**: Tutorial / reference node showing how to subscribe to simulator topics and publish velocity commands.
- **Files**: `src/basic_dev.cpp`, `include/basic_dev.hpp`
- **Key detail**: Uses `image_transport` for camera streams. Most sensor callbacks are commented out in the default build.

#### `airsim_ros`
- **Role**: Message and service definitions for the AirSim ROS bridge.
- **Custom msgs**: `VelCmd`, `RotorPWM`, `PoseCmd`, `GPSYaw`
- **Custom srvs**: `Takeoff`, `Land`, `Reset`

#### `imu_gps_odometry`
- **Role**: Error-State Kalman Filter (ESKF) fusing IMU propagation with GPS pose corrections.
- **Files**: `src/imu_gps_odometry.cpp`, `src/eskf.cpp`, `include/eskf.hpp`
- **Output**: Publishes `/eskf_odom` at IMU rate after initialization via `/airsim_node/initial_pose`.
- **Parameter tuning**: Hardcoded noise stddevs in `main()` (gravity, position/velocity/angle uncertainty, IMU biases, GPS noise).

#### `odometry`
- **Role**: Stereo visual odometry (VO) node.
- **Files**: `src/odometry.cpp`, `src/keyframe.cpp`, `src/mapmanager.cpp`
- **Algorithm**: ORB-like feature tracking with `cv::calcOpticalFlowPyrLK`, g2o `SparseOptimizer` for PnP pose refinement, keyframe-based map management.
- **Calibration**: Camera intrinsics, distortion, and stereo baseline are **hardcoded** in `odometry.cpp` (approx 640×480 stereo rig, baseline ~0.3m).

#### `controller` (package name `controller_test`)
- **Role**: Closed-loop trajectory follower using MPC + low-level rate controller.
- **Files**:
  - `src/controllerTest.cpp` — Main MPC trajectory follower. Loads a text file of waypoints, calls `rpg_mpc::MpcController::run()`, converts `ControlCommand` to `RotorPWM`, and publishes.
  - `src/PDcontroller.cpp` — Simpler PD controller (alternative).
- **Key parameters** (hardcoded defaults, overridable via `rosparam`):
  - Drone mass: `0.9 kg`
  - Arm length: `0.18 m`
  - Inertia: `Ixx=0.004689`, `Iyy=0.006931`, `Izz=0.010421`
  - Motor thrust coeff `Ct=0.000367717`, torque coeff `Cq=4.888e-06`
  - Max thrust per rotor: `12.538 N`
  - Control rate: `100 Hz`
- **Trajectory file**: Loaded from `trajectory_file` parameter (default hardcoded absolute path in the source). Format: plain text with `x y z` per line.
- **Motor index mapping**: `rotorPWM0=RF`, `rotorPWM1=LR`, `rotorPWM2=LF`, `rotorPWM3=RR`.

#### `rpg_mpc`
- **Role**: Model Predictive Control library.
- **Structure**:
  - `model/quadrotor_mpc_codegen/` — ACADO-generated C code for the quadrotor model and QP transcription.
  - `externals/qpoases/` — qpOASES solver source (embedded, no external install needed).
  - `include/rpg_mpc/mpc_controller.h`, `mpc_wrapper.h`, `mpc_params.h` — C++ Eigen wrapper and controller interface.
- **Important**: To modify the model or horizon, you must reinstall ACADO and regenerate the solver code. The generated code is already checked in, so standard builds do not require ACADO.

#### `rpg_quadrotor_common`
- **Role**: Shared type definitions and math utilities.
- **Packages**: `quadrotor_common` (types), `quadrotor_msgs` (message definitions), `state_predictor` (state estimation helpers).

---

## Development Conventions

### Language & Comments
- **LIO-SAM** and **EGO-Planner** codebases are entirely English.
- **RMUA2026-dev** source files contain a mixture of English and **Chinese comments** (especially `basic_dev.cpp`, `odometry.cpp`, `imu_gps_odometry.cpp`). Do not assume all comments are English.
- Documentation files in `RMUA2026-dev/` are mostly Chinese (`README.md`, `RMUA-codebase-doc.md`, `RMUA-simulator-doc.md`). Use `project-description-short.md` and `manual_compile.md` for English summaries.

### Coding Style
- No enforced linter or formatter is configured for the RMUA code.
- `ego-planner-raw/src/uav_simulator/mockamap/src/.clang-format` exists but only covers that single package.
- LIO-SAM uses C++11 with explicit `CMAKE_CXX_FLAGS "-std=c++11"`.
- RMUA controller uses C++17 (`set(CMAKE_CXX_STANDARD 17)`).

### Hardcoded Values
- Many physical drone parameters, camera calibration matrices, and file paths are **hardcoded** in source files rather than loaded from ROS parameters or YAML configs.
- The trajectory file path in `controllerTest.cpp` contains an absolute path from the original developer's machine. This must be changed when deploying on a new system.

### Git
- The repository is on branch `main`.
- The `-raw/` directories contain their upstream git histories as nested `.git` folders (subtree-style imports).
- Do **not** run `git commit`, `git push`, `git reset`, or `git rebase` unless explicitly requested.

---

## Testing Instructions

There are **no formal unit tests** in this repository. All validation is done through simulator integration tests.

### Manual Test Workflow

1. **Start the simulator** (obtained separately from the official RMUA2026 repository):
   ```bash
   cd /path/to/simulator_12.0.0.5
   ./run_simulator.sh 123        # 123 = random seed
   ```

2. **Build and source the workspace**:
   ```bash
   cd RMUA2026-dev/basic_dev/
   source /opt/ros/noetic/setup.bash
   catkin_make --only-pkg-with-deps airsim_ros imu_gps_odometry controller_test
   source devel/setup.bash
   ```

3. **Run nodes** (each in its own terminal, or use the Docker container):
   ```bash
   # Terminal A: ESKF odometry
   rosrun imu_gps_odometry imu_gps_odometry

   # Terminal B: MPC controller
   rosrun controller_test controller_test
   ```

4. **Monitor**:
   ```bash
   rostopic echo /airsim_node/drone_1/debug/pose_gt
   rostopic echo /eskf_odom
   ```

### Competition Submission Constraints
- The submission must be a **Docker image** (`.tar` export).
- The container must **auto-start** all nodes via its entrypoint.
- **Do NOT** set `ROS_IP` or `ROS_MASTER_URI` inside the image; the competition server assigns them externally.
- **No GUI** (X11, RViz, etc.) is allowed in the submitted image.
- The ground-truth topic `/airsim_node/drone_1/debug/pose_gt` is **unavailable** on the competition server; it exists for local debugging only.

---

## Deployment Process

1. Develop and test locally with the simulator running on the host.
2. Update `RMUA2026-dev/basic_dev/setup.bash` to launch the correct node combination for the final pipeline.
3. Build the Docker image:
   ```bash
   cd RMUA2026-dev/basic_dev/
   docker build -t basic_dev .
   ```
4. Export the image:
   ```bash
   docker image save basic_dev > test.tar
   ```
5. Submit `test.tar`.

---

## Security Considerations

- The project runs inside Docker with `--net host` during local testing, which exposes all host network interfaces. Only use this mode in trusted local environments.
- The `setup.bash` entrypoint runs ROS nodes as root inside the container (the Dockerfile does not create an unprivileged user).
- No secrets, API keys, or credentials are stored in this repository.
- The simulator and controller communicate over unencrypted ROS TCP connections. This is acceptable for the competition environment but should not be used over untrusted networks.

---

## Known Pitfalls & Critical Notes

1. **LIO-SAM point cloud requirements**: AirSim's LiDAR does not natively output `ring` or `time` per point. An adapter node must inject these fields before publishing to `/points_raw`. Without them, `imageProjection.cpp` will fail or produce incorrect deskewing.

2. **Frame conventions**: AirSim uses NED; LIO-SAM and EGO-Planner expect ENU (ROS REP-105). Coordinate transforms must be applied at the adapter layer.

3. **IMU extrinsics in LIO-SAM**: `config/params.yaml` defines `extrinsicRot` and `extrinsicRPY`. For the RMUA simulator, these must be recalibrated to match the simulated IMU-to-LiDAR transform.

4. **Hardcoded paths**: `controllerTest.cpp` contains an absolute path to the trajectory file. This must be parameterized or changed before any deployment.

5. **ESKF initialization**: `imu_gps_odometry` waits for `/airsim_node/initial_pose` to initialize the ESKF. If that topic is not published, no odometry is output.

6. **No loop closure in VO**: The `odometry` package is a visual odometry node without place recognition or loop closure. Long trajectories will accumulate drift.

7. **Clock sync**: The simulator clock differs from the host system clock. The official docs recommend using the **IMU message timestamp** as the global time reference.

---

## File Quick Reference

| File | Purpose |
|------|---------|
| `RMUA2026-dev/basic_dev/Dockerfile` | Competition submission image build |
| `RMUA2026-dev/basic_dev/setup.bash` | Container entrypoint (auto-starts nodes) |
| `RMUA2026-dev/manual_compile.md` | Host-side build/run cheat sheet |
| `project-description-short.md` | English integration plan & topic alignment |
| `LIO-SAM-raw/config/params.yaml` | LIO-SAM sensor/extrinsic parameters |
| `ego-planner-raw/src/planner/plan_manage/launch/advanced_param.xml` | EGO-Planner planner parameters |
| `ego-planner-raw/src/planner/plan_manage/launch/run_in_sim.launch` | EGO-Planner simulation launch |
