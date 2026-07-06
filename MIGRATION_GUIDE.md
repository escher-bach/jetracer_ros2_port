# JetRacer ROS 1 → ROS 2 Migration Guide & Roadmap

This document explains (1) how this repository differs from the original ROS 1 package
(`jetracer_ros`, Melodic, bare-metal on JetPack), and (2) what still needs to change —
both to fix known weaknesses and to get camera SLAM working on the Jetson Nano.

---

## Part 1 — What changed from the ROS 1 package

### 1.1 Deployment model

| | ROS 1 (original) | ROS 2 (this repo) |
|---|---|---|
| ROS distro | Melodic, bare-metal on JetPack (Ubuntu 18.04) | Humble, inside Docker (`ros:humble`, Ubuntu 22.04 userspace) on a JetPack 4.6 host |
| Remote access | Plain ROS 1 networking (`ROS_MASTER_URI`) | Zenoh bridge container (`zenoh-bridge-ros2dds`) exposing topics over TCP, with per-robot namespacing via `.env` |
| GPU access | Native — nodes could touch CUDA/ISP directly | **None inside the container.** GPU/ISP work runs on the host and results cross into the container via shared memory (see camera below) |

The GPU split is the single most important architectural consequence of the port:
the Humble container cannot run CUDA on JetPack 4.6 (18.04 L4T userspace vs. 22.04
container userspace). Anything GPU-accelerated must run host-side and communicate
through `/tmp` (mounted into the container).

### 1.2 Robot driver (`jetracer.cpp`)

Ported nearly 1:1. Same serial protocol, same topics (`/odom_raw`, `/imu`,
`motor/*`), same steering-calibration coefficients (`coefficient_a..d`,
`linear_correction`) with identical defaults, same hardcoded sensor covariances
(see §2.1 — these were bad in ROS 1 and are still bad).

Improvements in the port:
- Runtime parameter updates via a ROS 2 parameter callback (replaces ROS 1
  `dynamic_reconfigure` + `cfg/jetracer.cfg`).
- Divide-by-zero guard on the twist computation (`dt > 0`), which the ROS 1 node lacked.

### 1.3 Odometry fusion

| | ROS 1 | ROS 2 |
|---|---|---|
| Filter | `robot_pose_ekf` (deprecated, black-box) | `robot_localization` `ekf_node` |
| Odom input | Pose (x, y, yaw), differenced internally | Twist only: `vx` + `vyaw` |
| IMU input | Full orientation (roll/pitch/yaw) | Absolute yaw only |
| Fusion semantics | All inputs differential (deltas between updates) by design | Odom differential-equivalent (velocities); IMU yaw **absolute** — a semantic change, see §2.1 fix 3 |
| Gyro rate used? | No | No |
| Output plumbing | `/odom_combined` + `odom_ekf.py` relay script to republish as `nav_msgs/Odometry` for RViz | EKF publishes `/odom` directly (relay script obsolete, correctly dropped) |

The move to `robot_localization` with velocity fusion is the recommended modern
pattern and strictly more tunable. However, the *configuration* is under-informed —
see §2.1.

### 1.4 CSI camera

| | ROS 1 | ROS 2 |
|---|---|---|
| Pipeline location | `gscam` ran `nvarguscamerasrc` in-process (native GPU access) | Producer runs **on the host** via `nsenter` (`nvarguscamerasrc → nvvidconv → shmsink`), `gscam` in the container reads from the shm socket |
| Source mode | 1280×720 @ 60 fps, target 20 fps | 1280×720 @ 30 fps, 30 fps end-to-end |
| Output resolution | 680×420 | 680×420 (unchanged) |
| Calibration | `cam_640x480.yaml` (already mismatched with the 680×420 stream) | Same file, same mismatch — **inherited bug**, see §2.2 |
| Extras | `set_camera_info` service remap | JPEG-compressed transport with tunable quality (for the Zenoh link) |

The shm/nsenter producer-consumer split is new and is the pattern to reuse for any
future host-side GPU work (e.g. a depth net — see §2.6).

### 1.5 LiDAR and laser filtering

- Driver: `rplidar_ros` → `sllidar_ros2` (same Slamtec A1, same `/dev/ttyACM1`,
  same `base_footprint → laser_frame` transform: `z 0.1, yaw 3.14`).
- Filter: custom `laser_filter.py` script → the standard `laser_filters`
  `scan_to_scan_filter_chain` with `chassis_filter.yaml` (crops the chassis out of
  the scan). Output topic `/scan_filtered` consumed by SLAM, AMCL, and both costmaps.

### 1.6 SLAM

ROS 1 offered four selectable backends (`gmapping`, `hector`, `karto`,
`cartographer` — the `.lua` config is still in the old repo). The port
standardizes on **SLAM Toolbox** (async, Ceres solver), tuned for the Nano's CPU
(node-insertion gating at 0.5 m / 0.5 rad, 2 s map updates, 8 m range cap). This is
the right call for Humble; none of the ROS 1 options have healthy ROS 2 ports except
Cartographer, which is heavier and unmaintained.

### 1.7 Navigation

| | ROS 1 | ROS 2 |
|---|---|---|
| Framework | `move_base` | Nav2 |
| Local planner | TEB (`teb_local_planner`, car-like config) | Regulated Pure Pursuit, with Ackermann constraints (`use_rotate_to_heading: false`, `regulated_linear_scaling_min_radius: 0.50`, `allow_reversing: true`) |
| Global planner | `global_planner/GlobalPlanner` (grid A*) | Smac Planner Hybrid with `REEDS_SHEPP` motion model and matching 0.50 m turning radius — a genuine upgrade: global plans are now kinematically feasible for the car |
| Localization (saved map) | `map_server` + AMCL (tuned via `amcl.launch`) | `map_server` + AMCL — **but the params file only sets frame names; every motion/laser-model parameter is at diff-drive defaults** (see §2.4) |
| Recoveries | `clearing_rotation_allowed: false` | `wait`, `spin`, `backup` — **`spin` should not be here** (see §2.3) |

All Nav2 frequencies were deliberately reduced (controller 5 Hz, costmaps 0.5–2 Hz,
BT 20 Hz) to fit the Nano's 4× Cortex-A57 budget. Keep this in mind before adding
any new consumer of CPU.

### 1.8 Dropped features (deliberate or not yet ported)

- **Voice/audio stack** (`asr`, `tts`, `aiui`, `vad`, `iat`, `talk`, `audio_stream`) — Waveshare demo extras, not ported.
- **`calibrate_linear.py`** — the routine that drives 1 m and computes `linear_correction`. Not ported; `linear_correction` has consequently never been calibrated (still 1.0). Worth reviving (§2.1).
- **`multipoint_nav.py`** — waypoint sequencing; Nav2 has `FollowWaypoints` built in, so a thin replacement is easy if needed.
- **`capture`/`play`** launch files (rosbag helpers) — use `ros2 bag`.
- **`odom_ekf.py`** — obsolete relay, correctly dropped.

---

## Part 2 — What still needs to change

Ordered so that each item builds on the previous ones. Items A–D fix the existing
stack; E–F are the camera-SLAM roadmap.

### 2.1 (A) Fix the odometry fusion — highest value per line changed

The EKF currently behaves as "wheel odometry with an IMU-flavored heading" because
of three configuration facts:

> **Status:** fixes 2 and 3 are applied (twist `vx` covariance → `1e-3`,
> `imu0_differential: true`). Fix 1 (gyro fusion) is **deliberately deferred**: the
> `1e6` gyro covariance dates back to the original Waveshare driver and we don't
> know *why* it was disabled (genuinely bad/noisy part, wrong scaling, or just
> upstream laziness). Before fusing it, log `/imu` angular_velocity.z on the bench —
> stationary (bias/noise floor) and during a known rotation (scale check) — and
> compare against the MCU's integrated yaw over the same interval.

1. **The gyro is discarded.** *(Inherited from ROS 1 verbatim.)* The driver
   publishes `angular_velocity_covariance z = 1e6` ("never use") — identical in the
   ROS 1 driver — and `ekf.yaml` fuses no angular velocity from the IMU. Yaw rate
   instead comes from wheel odometry (`odom0 vyaw: true`) — the signal that lies
   exactly when an Ackermann car slips (cornering). The MEMS gyro z-rate is the most
   trustworthy motion signal on this robot. Note the port actually leans *harder* on
   wheel heading than ROS 1 did: `robot_pose_ekf` was fed odom yaw with covariance
   `1e3` (heavily distrusted), while the current config fuses wheel `vyaw` at `0.1`.
   - In `jetracer.cpp`: set gyro z variance to an honest ~`1e-3`–`1e-2`.
   - In `ekf.yaml`: fuse `vyaw` from `imu0` (set `imu0_config` element for vyaw
     to `true`); set `odom0` `vyaw` to `false` (or keep both with honest covariances).
2. **`vx` covariance is fiction.** *(Inherited — load-bearing in both eras.)*
   `1e-9` (σ ≈ 30 µm/s) claimed by wheel encoders on an RC car. The ROS 1 driver
   had the byte-identical blocks in both `pose.covariance` and `twist.covariance`;
   `robot_pose_ekf` consumed the pose one, `robot_localization` consumes the twist
   one — either way the filter has always been told to trust wheel-x blindly.
   Use ~`1e-3` (σ ≈ 3 cm/s) so the filter can actually arbitrate.
3. **Absolute IMU yaw fusion.** *(Port-introduced semantic regression.)*
   `robot_pose_ekf` fuses every input differentially by design, so ROS 1 consumed
   the MCU's integrated yaw as *increments* — effectively a rate signal. The current
   config (`imu0` yaw = true, `imu0_differential: false`) fuses it as an *absolute*
   heading. Steady-state drift passes through the same either way (differencing an
   integrated signal reproduces its increments, drift included), but absolute fusion
   adds a new failure mode: any MCU yaw discontinuity (board re-init mid-run, bad
   serial frame, integrator wrap/reset) yanks the entire odom-frame heading instead
   of being absorbed as one bad delta. Once the raw gyro rate is fused (fix 1),
   either drop absolute yaw or set `imu0_differential: true` — the latter restores
   the ROS 1 semantics.

Also:
- Add an explicit `process_noise_covariance` to `ekf.yaml` (currently running on
  generic robot_localization defaults).
- Port `calibrate_linear.py` (a ~50-line rclpy script) and run it once to set
  `linear_correction` for real.

### 2.2 (B) Fix the camera pipeline calibration — prerequisite for anything visual

- **Resolution mismatch (inherited from ROS 1):** the stream is 680×420 but
  `cam_640x480.yaml` was calibrated at 640×4xx. Every consumer of
  `camera_info` gets wrong intrinsics. Either recalibrate at 680×420 or change the
  pipeline caps to match the calibration — then rename the file to match reality.
- **Lens model:** the JetRacer lens is ~160° FoV with heavy distortion
  (k1 ≈ −0.34). For visual SLAM, recalibrate with a fisheye/equidistant model,
  or accept edge distortion and rectify centrally with `image_proc`.
- **Missing TF:** `frame_id: camera_frame` is published on the image topics but no
  one broadcasts `base_footprint → camera_frame`. Add a static transform in
  `jetracer_launch.py` (measure the camera position/tilt), plus the standard
  optical-frame rotation (`camera_frame → camera_optical_frame`,
  RPY `(-π/2, 0, -π/2)`) so visual packages get REP-103-compliant axes.
- On-device consumers (SLAM) must subscribe to the **raw** topic; the JPEG path
  exists for the Zenoh remote link only.

### 2.3 (C) Small Nav2 corrections

- Remove `spin` from `behavior_plugins` in `nav2_params.yaml` — an Ackermann car
  cannot rotate in place; if the BT ever invokes it, the robot stalls or grinds
  the steering. Keep `backup` (reversing is allowed) and `wait`.

### 2.4 (D) Fix localization before adding camera SLAM

Two options, in order of preference:

1. **SLAM Toolbox localization mode** (recommended): serialize a map
   (`ros2 service call /slam_toolbox/serialize_map ...`), then run
   `localization_slam_toolbox_node` with `mode: localization` and
   `map_file_name` pointing at the `.posegraph`. Reuses the existing tuning,
   generally outperforms AMCL on this class of robot, and makes
   `localization_launch.py`'s AMCL path optional.
2. **Or tune AMCL** — `amcl_params.yaml` currently sets *only* frame names and the
   scan topic; every other parameter is a diff-drive default. Minimum set for an
   Ackermann car with velocity-only odom: raise `alpha1`–`alpha4` (odometry noise),
   set `robot_model_type: "nav2_amcl::DifferentialMotionModel"` explicitly (Nav2
   has no Ackermann model — inflated alphas are how you compensate), bound
   `max_particles` (CPU), and set `laser_max_range: 8.0` to match the A1.

Either way, benchmark before/after with the §2.1 EKF fixes — a good chunk of the
current "mediocre AMCL" is odometry-prior error during cornering.

### 2.5 (E) Camera SLAM — the natural integration

**Approach: RTAB-Map in scan + RGB hybrid mode** (`ros-humble-rtabmap-ros`, apt
binary for arm64). Key property: with external odometry, the camera is processed
at the loop-closure detection rate (~1 Hz), not framerate — so it fits the Nano's
remaining ~1.5 cores. It *replaces* SLAM Toolbox (both publish `map`):

- Inputs: `odom` (from the EKF), `/scan_filtered`, `image_raw` + `camera_info`
  (rectified via `image_proc`).
- Visual odometry **off** (`odom` topic supplied externally), `Kp/MaxFeatures`
  ~400, detection rate 1 Hz, grid built from the scan
  (`Grid/FromDepth: false` / `Grid/Sensor: 0`).
- Output: `/map` occupancy grid for Nav2 (unchanged), plus appearance-based loop
  closure and global relocalization — the capabilities the LiDAR-only stack lacks.
- Localization mode later: same node with `Mem/IncrementalMemory: false` against
  the saved `~/.ros/rtabmap.db`.

Prerequisites: §2.1 (odometry quality), §2.2 (calibration + TF). Sequencing:
get mapping parity with SLAM Toolbox first, then enable visual loop closures,
then evaluate localization mode against AMCL/slam_toolbox-localization.

What was **evaluated and rejected** for this hardware (Nano A02: one CSI port, no
container GPU, JetPack 4.6 ceiling):
- *Full visual SLAM (ORB-SLAM3, stella_vslam):* 2–3 cores for tracking + BA, no
  occupancy grid for Nav2, mono scale ambiguity, rolling-shutter + vibration. Demo, not nav.
- *VINS-Mono/Fusion:* needs 100–200 Hz well-timestamped IMU; ours is low-rate,
  receipt-stamped on a shared 115200 serial line.
- *Isaac ROS Visual SLAM:* requires JetPack 5+; the Nano is capped at 4.6 forever.
- *Stereo:* the A02 carrier has a single CSI connector.

### 2.6 (F) Optional later: host-side depth net over shm

If 3D perception (obstacles above/below the scan plane) becomes a requirement, the
existing camera producer pattern generalizes: host-side TensorRT (CUDA 10.2-era
model, FastDepth class) at 2–5 Hz → second shm socket → small ingest node →
`rgb + depth + camera_info` as an extra Nav2 obstacle layer (STVL), or RGB-D input
to RTAB-Map. Constraints to respect: ~600 MB–1 GB of the shared 4 GB RAM for the
CUDA context + engine, and older network architectures only (TensorRT 8.2).
This is an add-on, not the foundation — the LiDAR remains the geometric backbone.

---

## Suggested execution order

| # | Item | Effort | Risk to running stack |
|---|---|---|---|
| 1 | EKF/driver covariance fixes (§2.1 fixes 2–3) — **done**; gyro fusion deferred pending bench characterization | Small (config + 1 line C++) | Low — verify odom in RViz before/after |
| 2 | Remove `spin` behavior (§2.3) | Trivial | None |
| 3 | SLAM Toolbox localization mode (§2.4) | Small | None — additive launch path |
| 4 | Camera recalibration + TF (§2.2) | Half-day incl. checkerboard session | None |
| 5 | RTAB-Map hybrid mapping (§2.5) | Medium | Medium — replaces slam_toolbox at launch level; keep `slam_launch.py` as fallback |
| 6 | RTAB-Map localization mode vs. AMCL benchmark | Small | None |
| 7 | Depth-net sidecar (§2.6) | Large | Isolated (host-side) |
