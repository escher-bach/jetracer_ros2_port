# RTAB-Map SLAM on the JetRacer — Implementation & Tuning Guide

Target: RTAB-Map graph SLAM on the Jetson Nano (JetPack 4.6, Humble container),
fusing the **monocular CSI camera** (appearance / loop closure), the **RPLIDAR A1**
(geometry / occupancy grid / ICP refinement), and the existing **EKF odometry**
(wheel velocity + IMU yaw, `robot_localization`).

Status: **implemented** — `config/rtabmap_params.yaml`,
`launch/rtabmap_slam_launch.py`, the `rtabmap_slam` rosdep dependency, and the
DB volume are in place. The laser TF height fix is deliberately deferred
(see [§9](#9-change-status)).

---

## 1. What RTAB-Map can and cannot do with this sensor set

A monocular camera has no depth. RTAB-Map cannot do visual odometry or dense 3D
reconstruction from it. What it *can* do — and what this design uses:

| Sensor | Role in RTAB-Map |
|---|---|
| CSI camera (RGB only) | **Loop-closure detection** via bag-of-words appearance matching. Detects "I have been here before" even when lidar geometry is ambiguous (corridors, symmetric rooms). |
| RPLIDAR A1 (2D scan) | All metric geometry: occupancy grid, ICP refinement of odometry links, ICP computation of the loop-closure transform (since mono features have no depth), proximity detection. |
| EKF odometry (`/odom`) | Motion prior between graph nodes. Quality directly bounds how far apart loop closures can still be found. |

So the honest framing: this is **lidar SLAM with visual loop closure**, not
visual SLAM. That is a well-supported RTAB-Map configuration
(`subscribe_rgb=true`, `subscribe_depth=false`, `subscribe_scan=true`) and is a
genuine upgrade over slam_toolbox in loop-closure robustness, at the cost of CPU
and RAM. It replaces slam_toolbox as the `map -> odom` publisher — **never run
both at once**.

```
 gscam (/csi_cam_0/image_raw + camera_info)  ─┐
 sllidar + laser_filters (/scan_filtered)    ─┤→  rtabmap  →  map -> odom TF, /map grid
 jetracer_node → ekf (/odom, odom TF)        ─┘
```

---

## 2. TF verification checklist (do this before first mapping run)

The runtime TF tree comes **only** from the static publishers in
`launch/jetracer_launch.py`. The URDF in `description/` is reference material
sampled from CagriCatik/JetRacer-ROS2 — it is not loaded by any launch file.

Current runtime values vs. URDF-derived ground heights:

Provenance matters here: **only the camera TF was ported from the URDF.** The
laser TF is inherited from the pre-existing (working) 2D-SLAM setup, not derived
from the URDF.

| Frame | Runtime TF (launch) | URDF → ground height | Verdict |
|---|---|---|---|
| `camera_link` | x 0.115, z **0.11**, pitch = arg | 0.065 + 0.045 = **0.11** | ported from URDF; verify with ruler |
| `laser_frame` | z **0.10**, yaw = π | 0.065 + 0.100 = **0.165** | **height suspect — see below**; orientation confirmed correct |
| `base_imu_link` | z 0.02 | 0.065 + 0.020 = 0.085 | irrelevant (EKF uses yaw only) |
| `base_link` | identity with `base_footprint` | z 0.065 | harmless while nothing loads the URDF |

### Finding 1 — laser height (0.10) contradicts the URDF and the physical layout

In the URDF the laser sits **above** the camera: 0.165 m vs 0.11 m ground
height (~5.5 cm higher). The launch file's inherited 0.10 puts it *below* the
camera lens (0.11), which contradicts the physical build (lidar on the top
plate, above the camera mast).

This is a classic unaudited inherited constant: nothing that ran so far ever
consumed laser z — 2D slam_toolbox is height-blind — so the value was never
validated by working behavior. Expected truth ≈ 0.16–0.17 m.

**Ruler check:** measure floor → the middle of the lidar's rotating emitter
window (not the base of the unit). Also measure floor → camera lens center
(expect ≈ 0.11 m). Whichever numbers you measure win.

Impact if left wrong: none for 2D slam_toolbox; small for RTAB-Map as configured
here (grid and ICP are 2D). It matters the moment anything projects scan points
into the camera image or builds a 3D cloud — so fix it while it's cheap.

### Finding 2 — laser orientation: RESOLVED, launch value is correct

The launch's yaw = π is confirmed correct (verified against real scans; also
consistent with slam_toolbox having produced usable, non-mirrored maps). The
URDF's roll = π (upside-down mount) is wrong for this car — one more reason to
treat `description/` as untrusted reference, not ground truth.

### Finding 3 — camera forward offset and pitch

- `x = 0.115` is measured from the **midpoint between the axles** (the
  `base_footprint` origin). The front axle is at +0.1275, so the lens should sit
  ~1 cm behind the front axle. Verify: measure rear axle → lens horizontally;
  expect ≈ 0.1275 + 0.115 = 0.2425 m.
- **Camera pitch** is a launch argument defaulting to 0, but the mount is a
  hinge. For loop-closure matching it mostly cancels out (same tilt both
  visits), but an unmeasured tilt degrades feature stability and any future
  scan↔image cross-checks. Lock the hinge (tape/mark it) and measure the angle.
  Note: the comment in `jetracer_launch.py` references `scripts/scan_overlay.py`
  — that script does not exist in the repo on any branch.

### TF sanity commands (inside the container, stack running)

```bash
ros2 run tf2_tools view_frames                       # one connected tree, no duplicates
ros2 run tf2_ros tf2_echo base_footprint laser_frame # z and yaw as decided above
ros2 run tf2_ros tf2_echo base_footprint camera_link_optical
ros2 topic echo /csi_cam_0/camera_info --once        # frame_id: camera_link_optical
ros2 topic echo /scan --once --field header          # frame_id: laser_frame
```

---

## 3. Installation

Via rosdep, not a Dockerfile apt line. In `src/jetracer_ros2/package.xml`:

```xml
<exec_depend>rtabmap_slam</exec_depend>
```

The Dockerfile already copies `package.xml` early and runs
`rosdep install --from-path src`, so this resolves to
`ros-humble-rtabmap-slam` (arm64 binaries exist; no source build) in the
existing cached layer — no Dockerfile change needed.

Depend on `rtabmap_slam` (just the SLAM node), **not** the `rtabmap_ros`
metapackage: the metapackage drags in `rtabmap_viz` and its Qt stack, which is
dead weight on the Nano. Visualize from the remote machine instead.

The map database persists across container restarts via a volume in
`docker-compose.yml`:

```yaml
    volumes:
      - ./maps/rtabmap:/data/rtabmap   # database_path: /data/rtabmap/rtabmap.db
```

---

## 4. Node configuration (the core of the setup)

**Parameter policy:** trust RTAB-Map defaults. A parameter earns a place in the
config only if it is (a) *required* — the mono+scan topology doesn't function
without it, (b) *standard* — set the same way in RTAB-Map's own reference
config for 2D-lidar robots (the upstream TurtleBot3 `rtabmap.launch.py` demo),
(c) *already proven in this repo* (carried from the working slam_toolbox
config), or (d) *known embedded-performance settings* — upstream's own guidance
for RPi/Jetson-class CPUs, since the defaults assume a desktop. Everything else
stays at default and is touched only from the tuning section, one knob at a
time, in response to an observed symptom. A sprawling parameter list makes
tuning impossible — you can never tell which knob did what.

To see what any default actually is on the installed version:

```bash
ros2 run rtabmap_slam rtabmap --params | grep <ParamName>
```

Parameters live in a YAML under `config/`, same pattern as
`slam_toolbox_params.yaml` — standard ROS 2 practice, and it keeps tuning
history visible in git diffs instead of buried in launch code. (Many upstream
rtabmap examples inline a dict in the launch file; that works but diffs badly.)

`config/rtabmap_params.yaml`:

```yaml
rtabmap:
  ros__parameters:
    # ---- (a) REQUIRED: topology / plumbing ----
    frame_id: base_footprint
    subscribe_rgb: true          # mono camera: RGB without depth
    subscribe_depth: false
    subscribe_scan: true
    qos_scan: 2                  # sllidar publishes BEST_EFFORT; without this
                                 # the scan subscription gets no data
    database_path: /data/rtabmap/rtabmap.db   # persisted volume (§3)

    # NOTE: RTAB-Map library params (the Slash/Named ones below) are STRING
    # parameters — keep the quotes ("1", "true"), or the node rejects them.

    # ---- (a) REQUIRED: mono camera cannot compute transforms ----
    Reg/Strategy: "1"            # ICP: camera *detects* loops, scan *computes*
                                 # the transform (no depth)
    Grid/Sensor: "0"             # grid from scan; default (1) expects a depth
                                 # camera we don't have
    RGBD/LoopClosureIdentityGuess: "true"   # without depth, no node has 3D visual
                                 # features, so the default visual transform-guess
                                 # stage rejects EVERY closure with "Not enough
                                 # features in images (old=0)"; identity guess
                                 # sends closures straight to scan ICP

    # ---- (b) STANDARD: upstream 2D-lidar reference config (TurtleBot3 demo) ----
    Reg/Force3DoF: "true"                   # ground vehicle, 2D
    RGBD/NeighborLinkRefining: "true"       # refine odom links with scan ICP
    RGBD/ProximityBySpace: "true"           # scan-based proximity re-localization
    RGBD/ProximityPathMaxNeighbors: "10"    # default 0 = scan proximity off

    # ---- (c) PROVEN IN THIS REPO ----
    Grid/RangeMax: "8.0"         # = max_laser_range in slam_toolbox_params.yaml

    # ---- (e) LOOP-CLOSURE GATES relaxed for sparse A1 + drifty odom (§6) ----
    Icp/MaxTranslation: "0.4"    # default 0.2; A1 drift at loop time > 20 cm
    RGBD/OptimizeMaxError: "4.0" # default 3.0; sparse-scan ICP overconfident,
                                 # inflates error ratio on CORRECT closures
    Icp/CorrespondenceRatio: "0.06"  # default 0.10; A1 repeatedly measured
                                 # 0.055-0.092 overlap on valid geometry

    # ---- (d) EMBEDDED PERFORMANCE (upstream RPi/Jetson guidance) ----
    Rtabmap/TimeThr: "700"       # ms; bounds map-update time by moving old nodes
                                 # WM->LTM (default 0 = unbounded, desktop-sized)
    Kp/MaxFeatures: "200"        # BoW features/image (default 500, desktop-sized)
```

(`Kp/DetectorStrategy` is deliberately left at its GFTT+ORB default: the
cheaper GFTT+BRIEF option needs OpenCV xfeatures2d, which the Humble binaries
lack — setting it just logs warnings and falls back anyway. Verified on-robot.)

And the thin launch node in `launch/rtabmap_slam_launch.py` — it includes
`slam_launch.py`'s robot+lidar parts **minus slam_toolbox**, plus the camera
and:

```python
Node(
    package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
    parameters=[rtabmap_params_path],
    remappings=[
        ('rgb/image',       '/csi_cam_0/image_raw'),
        ('rgb/camera_info', '/csi_cam_0/camera_info'),
        ('scan',            '/scan_filtered'),
        ('odom',            '/odom'),
        ('grid_map',        '/map'),   # Nav2 expects /map
    ],
    arguments=['-d'],   # delete old DB on start — REMOVE once you map incrementally
)
```

Thirteen RTAB-Map parameters, each traceable to a category. Deliberately **left
at default** (change only via §6, symptom first): `Rtabmap/DetectionRate`
(default is already 1 Hz), `Kp/DetectorStrategy` (see above), the other
`Icp/*`, `Mem/*`, `Rtabmap/LoopThr`, `approx_sync` (default true),
`sync_queue_size`.

Category (e) was added empirically on-robot: with `LoopClosureIdentityGuess`
working, ICP started computing real loop-closure transforms but they were
rejected by gates tuned for dense desktop scanners — `Icp/MaxTranslation`
(correct >20 cm corrections capped), `RGBD/OptimizeMaxError` (correct ~1.4°
closures rejected at ratio 4.29 because sparse A1 ICP reports an overconfident
covariance), and `Icp/CorrespondenceRatio` (valid geometry repeatedly measured
at 0.055-0.092 overlap, below the 0.10 default). Landing loop closures is what
heals an odometry yaw jump — see the ghost-lab note in §6.3. Trade-off: a lower
correspondence-ratio floor also raises the chance ICP converges on ambiguous
geometry (corridors, symmetric rooms) — watch for map folding, not just for
more accepted closures.

### Odometry covariance caveat

RTAB-Map reads covariance from `/odom`. `robot_localization` outputs a full
covariance matrix, so this works. If loop closures are rejected with
`OptimizeMaxError` messages, the odometry covariance may be optimistic relative
to real drift — see §6.3.

---

## 5. First-map workflow

1. `docker compose up`, exec in, launch the rtabmap stack (once created).
2. On the remote machine (zenoh bridge running): RViz with `/map`, `/scan`, TF;
   or `rtabmap_viz` if you bridge the `/rtabmap/*` topics.
3. Check the console: rtabmap warns loudly if inputs don't sync
   (`Did not receive data since 5 seconds!` — see §7).
4. Drive **slowly** (< 0.5 m/s), smooth arcs, no handbrake turns. Ackermann +
   30 fps rolling-shutter CSI camera = motion blur kills features.
5. Deliberately close loops: return to the start area facing the **same
   direction** — a mono camera cannot loop-close a corridor traversed the
   opposite way (appearance differs 180°). Lidar proximity detection can still
   catch some reverse loops.
6. Watch for `Loop closure detected!` in the log. After a good loop closure the
   map visibly snaps into alignment.
7. Save: the DB is the map (`/data/rtabmap/rtabmap.db`). For a Nav2 static map:
   `ros2 run nav2_map_server map_saver_cli -f maps/rtabmap_map` while running.

---

## 6. Tuning guide

Tune in this order — each layer depends on the one before.

### 6.1 Input health (before any parameter tuning)

| Check | Command | Want |
|---|---|---|
| Image rate | `ros2 topic hz /csi_cam_0/image_raw` | ~30 Hz steady |
| Scan rate | `ros2 topic hz /scan_filtered` | 7–10 Hz (A1) |
| Odom rate | `ros2 topic hz /odom` | ~30 Hz (EKF frequency) |
| Stamp skew | `ros2 topic delay` on each | < 100 ms |
| Calibration | image undistorts sanely | see note below |

Calibration note: `config/cam_640x480.yaml` is actually 640×420 and matches the
gscam pipeline (which scales 1280×720 → 640×420, non-uniformly — fx≠fy absorbs
that). **If you ever change the pipeline resolution, recalibrate**; a wrong K
silently degrades BoW matching.

### 6.2 CPU / memory (Nano is the constraint)

Symptoms: rtabmap iteration time > 1 s in log, growing sync warnings, OOM.
Change **one knob at a time**, re-run the same test loop, compare.

| Knob | Baseline (§4) | Move toward |
|---|---|---|
| `Rtabmap/TimeThr` | 700 ms | 500 (moves nodes to LTM sooner) |
| `Rtabmap/DetectionRate` | 1 Hz (default) | 0.5 (halves everything downstream) |
| `Kp/MaxFeatures` | 200 | 150 (below ~100, loop closures starve) |
| `Mem/ImagePreDecimation` | 1 (default) | 2 (features on 320×210 — last resort) |

`TimeThr` is RTAB-Map's signature memory management: when an update exceeds it,
nodes move from working memory to long-term memory (still in DB, retrievable on
loop closure). It degrades gracefully — prefer it over hard node caps.

### 6.3 Loop closure quality

**The "ghost lab" failure — why loop closures matter more than they look.**
Symptom: driving straight is fine, then during a turn a rotated duplicate of an
already-mapped area appears and the session never recovers. Mechanism: RTAB-Map
trusts `/odom` between graph nodes (it does *not* continuously scan-match every
frame the way slam_toolbox does), so a single odometry yaw discontinuity — the
known MCU yaw jump absorbed by `ekf.yaml`'s differential IMU fusion, or wheel
slip in a hard Ackermann turn — places subsequent scans at a wrong heading and
duplicates the space. A **loop closure is the only thing that heals this**: it
lets the graph optimizer pull the ghost back onto the real map. So chronic
loop-closure rejection doesn't just "miss optimizations" — it removes the
system's only recovery from odom jumps, turning a transient glitch into a
session-ending derail. Critically, the camera (appearance/BoW) is the only
*odometry-independent* revisit detector: `RGBD/ProximityBySpace` searches by the
odom-predicted pose, so it fails exactly when odom has jumped. This is why the
camera is load-bearing here, not garnish — and why a camera outputting noise is
*worse* than none (it injects false candidates rather than degrading to clean
lidar SLAM). Durable fix is at the odom layer (lidar ICP odometry, or the driver
yaw-discontinuity bench-test item); the cheap fix is making closures land (below).

- **Too few closures:** first check driving pattern (loops facing the same
  direction) and lighting — BoW hates auto-exposure swings. Then, if CPU
  headroom allows, raise `Kp/MaxFeatures` back toward its 500 default (our 200
  baseline trades recall for CPU). Only then lower `Rtabmap/LoopThr`
  (default 0.11 → 0.09; below ~0.07 expect false positives).
- **Wrong closures accepted (map folds onto itself):** raise `Rtabmap/LoopThr`;
  tighten `RGBD/OptimizeMaxError` back toward the 3.0 default (our baseline is
  4.0 — see below); verify scan ICP params (a bad ICP transform on a correct
  visual detection also folds the map).
- **Closures detected but rejected** (`RGBD/OptimizeMaxError` in log, error ratio
  above the limit): the optimizer thinks the closure contradicts odometry. With
  sparse A1 scans this is usually an *overconfident ICP covariance* on an
  otherwise-correct closure (small absolute error, e.g. ~1–2°, but a high
  ratio) — the reason our baseline raises the limit to **4.0**. Distinguish from
  a genuinely wrong closure by the absolute error: tens of degrees = real ghost,
  keep it out; a couple degrees = calibration, let it in. Only suspect
  `ekf.yaml` covariance with bench data (odometry evidence standard).

### 6.4 Scan ICP (all at defaults initially)

- ICP failures in log (`Registration failed`): raise
  `Icp/MaxCorrespondenceDistance` (default 0.1 → 0.2–0.3); check the scan isn't
  mostly chassis returns (the box filter handles this — confirm
  `/scan_filtered` is the subscribed topic).
- ICP succeeds but refined poses jitter, or the map starts folding onto itself:
  raise `Icp/CorrespondenceRatio` back toward the 0.10 default (our baseline is
  0.06 — see §4(e)) so weak-overlap matches are rejected again; consider
  `Icp/VoxelSize 0` (default 0.05; 0 = no downsampling — A1 scans are already
  sparse).
- Featureless corridors: ICP slides longitudinally. That's what the camera
  loop closures are for; also `RGBD/ProximityBySpace` helps on re-traversal.

### 6.5 Grid map for Nav2

- `Grid/RangeMax 8` matches slam_toolbox's `max_laser_range`. Lower to 6 if the
  A1 returns noisy long-range points indoors.
- Ray-tracing of free space: `Grid/RayTracing true` if the grid shows unknown
  where it should show free.
- If Nav2 complains about map QoS, `map` from rtabmap is transient-local like
  slam_toolbox's — no change needed.

---

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Did not receive data since 5 seconds!` | topic names, QoS mismatch, or stamps too far apart for `approx_sync` | check remaps; `qos_scan: 2`; raise `sync_queue_size`; verify clocks (all in one container — should be fine) |
| Every closure rejected: `Not enough features in images (old=0, new=NNN)` | mono nodes have no 3D visual features, so the visual transform-guess stage can never pass — closures die before scan ICP runs | `RGBD/LoopClosureIdentityGuess: "true"` (in the config since 2026-07-07; this row documents the log signature) |
| Map is mirrored | laser orientation TF changed (current yaw = π is verified correct) | restore yaw = π; roll = π would mirror scans |
| Map -> odom TF missing | rtabmap not initialized (no data) or slam_toolbox also running | fix inputs; never run both SLAMs |
| Everything drifts, no closures ever | camera dark/blurry, or `Kp/MaxFeatures` starved | view `/csi_cam_0/image_raw` remotely; more light; slower driving |
| rtabmap killed (OOM) | WM too large | §6.2, and confirm `-d` isn't accumulating an old giant DB |
| Loop closes to wrong side of a symmetric room | visual aliasing | raise `Rtabmap/LoopThr`, add visual landmarks to the room, tighten `RGBD/OptimizeMaxError` |

---

## 8. Localization mode (after a good map exists)

Reuse the same launch with:

```python
'Mem/IncrementalMemory': 'false',   # localization only, DB read-only-ish
'Mem/InitWMWithAllNodes': 'true',   # load whole graph (small maps only, Nano RAM)
```

and **remove `-d`**. This replaces the AMCL path in `localization_launch.py`
(pick one; both publish `map -> odom`).

---

## 9. Change status

Applied:

- **package.xml**: `<exec_depend>rtabmap_slam</exec_depend>`; rosdep in the
  existing Dockerfile flow installs it (§3). Requires `docker compose build`.
- **docker-compose.yml**: `./maps/rtabmap:/data/rtabmap` volume (§3).
- **`config/rtabmap_params.yaml` + `launch/rtabmap_slam_launch.py`** as
  specified in §4 (robot + lidar + filter + camera + rtabmap, no slam_toolbox).

Deferred by decision:

- **Laser TF height** (`jetracer_launch.py` z 0.10, expected ≈ 0.165 — §2
  Finding 1): left as-is for now. Height-blind 2D SLAM is unaffected; revisit
  before anything projects scans into the camera or builds 3D clouds.
- Camera-pitch measurement script: dropped.

Resolved without change: laser orientation (yaw = π) confirmed correct.
