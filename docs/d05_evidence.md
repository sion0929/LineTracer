# D-05: real C++ OpenCV detector and Gazebo mesh-ground-truth evidence

Date: 2026-11-19, lab container ROS 2 Jazzy / Gazebo Harmonic / OpenCV 4.6.0.
Final command outcomes and complete measured tables appear below. Only final runs determine acceptance;
earlier intentional red tests and failed development experiments are explicitly listed separately.

## Reproduce

From the workspace, with the trailing space in the parent directory quoted:

```bash
cd '/home/sion/Line_Tracer /line_follower_ws'
tools/lab_env.sh build
tools/lab_env.sh test
tools/lab_env.sh check-d05
tools/lab_env.sh checkpoint-a
tools/lab_env.sh check-gcs
tools/lab_env.sh check-bag
tools/lab_env.sh check-camera-rate
docker build -t line-follower-d05:local .
```

For the reliable C++ camera observer used in the final 20 s CPU sustainability measurement:

```bash
docker exec line_follower_lab bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; PROBE_RELIABLE=1 RATE_COMPOSED_ONLY=1 python3 tools/check_camera_rate.py'
```

All ROS/Gazebo runs are container-side. Checkpoint A uses its existing domain/partition isolation;
GCS uses 82, recording/replay use 84/85, camera comparison uses 90/91, D-05 uses 92. Each simulator
gets a unique partition and owned launch process group. The provided 12 SHA256-guarded assets remain
unchanged; `design.md` checkboxes are untouched. Docker's unchanged production build selects only
`--packages-up-to line_follower_bringup`, Release, BUILD_TESTING=OFF; no Qt/GCS/dev-tool dependency
was added to that package set. The official competition one-liner and default composition remain.

## Part 0: external camera rate investigation and limited fix

The first comparative experiment ran with the **unchanged dummy detector**, four simultaneous
external observers (C++ SensorData subscriber, Python checker, Python CLI, rosbag), and a stationary
speed-capped competition profile. This was done before detector implementation, within the time box.
It was not just a `ros2 topic hz` display artefact: the original composed best-effort publisher lost
recorded images while its intra-process detector maintained full rate.

| Dummy pipeline experiment | Bag raw images / duration | Bag camera Hz | Bag detector status Hz | C++ SensorData Hz | Python checker Hz | CLI last reported Hz |
|---|---:|---:|---:|---:|---:|---:|
| Original standalone `ros_gz_image` | 611 / 20.123 s | 30.363 | 27.530 | 27.600 | 26.889 | 27.532 |
| Original composed SENSOR_DATA bridge | 490 / 20.136 s | 24.334 | 30.343 | 25.799 | 26.624 | 26.171 |
| Composed reliable bridge, same dummy backend | 611 / 20.130 s | 30.353 | 30.353 | 26.349 | 26.750 | 26.513 |

Raw JSON: `log_lab/d05-camera/before.json`, `after-reliable.json`; detailed original logs:
`log_lab/d05-camera-final.log`, `log_lab/d05-camera-services.log`.
The first C++ probe compile attempt used a generic ROS subscription lambda unsupported by the Jazzy callback traits; an explicitly typed callback fixed it (`d05-camera-initial.log`).

The small change is **RELIABLE / VOLATILE / KEEP_LAST, depth 5**, rather than best-effort SensorData
on the image-only RosGzBridge publisher. This SDK exposes it as its `SERVICES` QoS preset; the name
selects QoS properties and does not create a ROS service. Intra-process communication stays enabled,
volatile and KeepLast-compatible. Executor thread count stays 4. Reliable subscribers repair losses;
best-effort readers still match, but remain lossy under the simultaneous raw-image observer load.
The development acceptance checker now requests reliable raw images too. Other sensor QoS and every
safety threshold are unchanged.

A further dummy-backend reliable C++ reader measured 610 images over 20.001 s (~30.45 Hz), while the
simultaneous CLI still reported ~25.43 Hz. That recording's delayed discovery left only 17.610 s in
the bag, so it is **not** presented as the 20 s bag proof. The 20.130 s bag above is that proof. The
final real-detector measurement below independently repeats a full ~20 s recording.

Inference: the change in publisher reliability and the recovery by reliable readers implicate
loss/repair in the inter-process image transport, rather than executor starvation or detector input
rate. No packet capture was taken, so a specific kernel/FastDDS fragmentation fault is not claimed.
Jazzy's installed `ros2 topic hz --help` has no `--qos-profile` option; the independent C++ probe is
the explicitly controlled SensorData/reliable comparison. CLI rates remain observer-specific.

## Detector design and contracts

The C++ OpenCV baseline replaces the fixed path. It caps an Otsu dark threshold at 100 and adds a
31 px local black-hat contrast mask (>24 intensity, dark pixel <170) to retain faint one-pixel
transverse lines. No Gaussian smoothing/opening erases narrow strokes. The horizon is excluded.
A capture-attitude-aware inverse mapping produces a 4 mm ground grid over forward x=0.28–1.2 m and
lateral ±0.72 m, clipped by actual image coverage. Ground-grid components below 10 pixels are removed.
Zhang–Suen thinning yields a centerline graph. Junction clusters have explicit incident legs; graph
traces stop at junctions rather than selecting a bent path through an X. Polygon simplification
retains arcs; a short medial-axis chamfer between two long perpendicular legs is replaced with their
intersection, preserving sharp right-angle corners. Points are resampled along each edge, not across
corners. Connected IDs refer to actual incident segment IDs. Two-/three-/four-leg candidates encode
corners/T candidates/crossings in the unchanged message schema; only mission_manager decides finish.

Camera geometry remains parameterized: width/height 640/480, horizontal FOV 62.2 degrees, forward
offset 0.076 m, ground height 0.103 m. Small capture-time IMU pitch/roll rotate camera rays into the
flat, gravity-aligned ground projection. This matters: the actual settled flat robot has pitch
about -0.0057 rad (~-0.33 degrees). Ignoring it produced ~5 cm errors near the far edge. The node
uses the nearest stamped IMU sample within 50 ms, bounded history 400 samples; missing/invalid
quaternion or unsupported orientation is never treated as valid. It does not substitute callback
time for capture time. The message header and every observed point's capture stamp retain the input
image stamp; frame is `base_footprint`.

Missing capture-time attitude, absolute pitch/roll ≥2 degrees, or a large fraction of implausibly
broad projected strokes makes `projection_valid=false`, with diagnostic points retained and confidence
reduced from 0.9 to 0.25. These are heuristic quality values, not calibrated probabilities. The
planner stops and excludes such observations from observed graph/path memory; otherwise invalid
flat projections could have been reused as confirmed ground observations. The diagnostic coordinates
and ranges in an invalid observation are approximate only. No map or official track coordinate is
present in the detector/planner implementation. Full slope transition/memory handling remains D-07.
Fast-SCNN stays the existing unavailable D-15 stub with OpenCV fallback; no training/model comparison
is claimed.

`postprocess_ms` measures the actual image callback's steady-clock duration, including RGB/BGR
conversion, segmentation, graph/projection and line publication. `inference_ms` remains zero for
this non-neural backend. Outputs retain owned `std::unique_ptr` publication and the existing component
architecture. The lab build now selects Release, matching the production Docker build, for meaningful
CPU timing.

## Test-first evidence

`log_lab/d05-red.log`: synthetic tests were written first; compilation failed because `project_pixel`
and `set_attitude` did not exist. Subsequent synthetic runs exposed unsplit thick-stroke L corners;
short-chamfer intersection fixed this. Navigation red tests (`d05-nav-red.log`, XML results) exposed
that untrusted projections still became a drive path and entered observed memory, before those gates
were implemented. Initial Gazebo checks (`d05-initial.log`, `d05-second.log`) failed distance/recall
for faint transverse lines and ignored small pitch. The fixes above preceded the final green runs.

The first harness incorrectly included unrelated GT beyond the observation's reported range in
recall. It now uses the specified reported **radial range**, the camera FOV and flat GT; it does not
narrow the distance-error tolerances. Initial thresholds (25/45/80 mm median/p95/max, 80% recall at
30 mm, topology within 50 mm) remain unchanged. `qos_profile: DEFAULT` was an invalid SDK profile
experiment and failed startup; it was replaced by the supported preset and is not shipped.

There are 14 C++ backend gtests: Fast-SCNN fallback; baseline selection; empty input; no-line light
image; straight observed points; known-pixel ground projection; both sharp corner directions; X with
four connected legs; T marker candidate; arc plus image-edge line; slope invalidation while keeping
points; small capture-pitch correction; faint distant transverse stroke; missing attitude invalidation.
Additional navigation tests require invalid projection to stop and to stay out of observed memory.
The read-only mesh oracle has independent strip-width/length/distance/sampling/pose tests.

An additional real regression was found during the final rerun: GCS failed while unloading the
active line detector, and the container logged SIGSEGV. A node-only stress test reproduced exit 139
on the first unload while images arrived (`log_lab/d05-unload-red.log`). The detector's raw-this
callbacks could overlap node destruction after executor removal. The fix uses a shared lifetime gate:
the destructor waits for a running callback, marks the shared gate closed, and queued callbacks check
that gate before accessing the node. The gate itself outlives the node through callback captures.
No unload timeout was enlarged and no retry masks the failure. The current stress test covers 80
active detector unloads and 80 planner unloads; the original detector-only stress passed after the
fix (`d05-unload-fixed.log`). This additional test is in colcon test. The planner's callback
implementation was not changed because the stress test passed it; its projection safety gates above
remain the only navigation logic changes.

The final overlay projection initially omitted the small capture-time attitude when converting ground points back to pixels. A pure projection test was added first (`d05-overlay-red.log`), then the dev projection helper and visibility/overlay calls were corrected to use the matched IMU pitch/roll. This changes the diagnostic rendering/FOV calculation, not runtime perception. The unit suite and D-05 Gazebo check were rerun, followed by a fresh bag/replay check.

The camera diagnostic also found a metric error: dividing image count by the **whole bag** duration
can underestimate camera Hz when status discovery starts before image discovery. The failed run
recorded 611 images but its aggregate bag duration was 22.637 s. It now reads each topic's own first
and last bag timestamps and computes `(count-1)/topic_interval`, recording both topic intervals and
whole-bag duration. No rate threshold was lowered. The final camera diagnostic is rerun with this
corrected denominator.

The two node-only launch fixtures previously sent **white 64×48 images** and depended on the dummy
always reporting a line. They now send 640×480 images with a genuine black stroke and valid identity
IMU orientation. The GCS test waits for a READY snapshot containing the populated path. Existing
motion, timeout, zero-command, provenance and capture-stamp assertions were not weakened.

## Gazebo oracle and visual method

`tools/d05_groundtruth.py` reads all 806 binary STL triangles as 403 verified 2 cm strip quads,
extracts centerlines between short-edge midpoints, and applies world/model/link/visual transforms
from the unmodified SDFs. It imports no detector implementation. Named capture positions are derived
from mesh endpoints and offsets only in the dev tool. `tools/d05_capture.launch.py` starts sim,
bridges, detector and planner, with no mission/controller/final velocity publisher. The sole static
policy exception is the exactly named `tools/check_d05.py`, which invokes the owned Gazebo teleport
service; an additional policy test prevents rover source/config/launch from importing the harness
or oracle. Every other file remains under the scoped-interface prohibition.

Each pose gets a 2 s settle and a 4 s reception sample. Overlay image and observation are joined by
exact capture stamp. Distance and recall metrics use that final matched frame; rates, processing
quantiles, line-found/projection-valid fractions use the whole sample. Detected polyline points are
transformed by the known teleport pose to world XY and compared with independently extracted mesh
centerlines. Recall samples GT every 5 mm, restricts to visible flat GT inside the reported radial
range, and measures distance to detected polylines. Required landmark legs must match 3/2/4/3 for
start T/corner/X/finish T. Every flat frame must report line found and valid projection. The start T
is viewed from an offset pose: at the actual spawn its bar is beneath/behind the forward camera.

These are stationary flat-ground perception checks, not a full autonomous track drive, not real-world
calibration, and not proof of hill completion. The hill entry is information only; it has no valid
flat GT recall in this ROI. During settling it crosses the 2-degree pitch threshold, so early hill
frames can still be marked valid before the final invalid observation; D-07 must handle transitions.
Distant curved strokes can yield extra two-leg corner candidates; topology is still a candidate,
not a route choice. Landmark matching and geometric accuracy are gated, exhaustive candidate
false-positive precision is not measured here. The planner's staged branch-selection strategy remains.

## Final D-05 results

`tools/lab_env.sh check-d05` exited 0: stationary graph, six flat poses, informational hill and owned-group cleanup PASS.

| Pose | Result | Median / p95 / max error (m) | Recall | Segments; candidate leg counts | Range min / max (m) | Quality; projection | Processing p50 / p95 (ms) | Line Hz; frames |
|---|---|---|---:|---|---|---|---|---|
| start_t | PASS | 0.00089 / 0.00729 / 0.00800 | 1.000 | 3; [3] | 0.432 / 0.608 | 0.90; True | 3.12 / 6.19 | 30.25; 121 |
| straight | PASS | 0.00546 / 0.00864 / 0.02067 | 1.000 | 8; [2, 4, 2] | 0.316 / 1.373 | 0.90; True | 3.35 / 4.56 | 30.24; 121 |
| corner | PASS | 0.00573 / 0.00777 / 0.00800 | 1.000 | 3; [2] | 0.316 / 1.363 | 0.90; True | 3.13 / 4.65 | 30.25; 121 |
| curve | PASS | 0.00200 / 0.00544 / 0.00670 | 1.000 | 1; [] | 0.316 / 0.662 | 0.90; True | 2.89 / 4.29 | 30.48; 122 |
| crossing | PASS | 0.00400 / 0.00800 / 0.01836 | 1.000 | 5; [4, 2] | 0.316 / 1.363 | 0.90; True | 3.22 / 4.69 | 30.25; 121 |
| finish_t | PASS | 0.00000 / 0.00303 / 0.00400 | 1.000 | 3; [3] | 0.316 / 0.531 | 0.90; True | 2.90 / 3.92 | 30.49; 122 |
| hill_entry | INFO (nongated) | 0.00000 / 0.08276 / 0.08400 | 0.000 | 2; [] | 0.344 / 1.329 | 0.25; False | 3.32 / 4.45 | 30.24; 121 |

Every flat sample has line_found_fraction=1.0 and projection_valid_fraction=1.0. Hill final pitch=-0.04652 rad; final projection invalid, quality 0.25; projection_valid_fraction=0.463 during settling. Hill does not enter the flat gate.

Raw data: [`d05/measurements.json`](d05/measurements.json); exact rows: `log_lab/d05-check-d05-final.log`. Maximum flat representative-frame error 0.02067 m.

### Flat-ground column for design §14.2 (evidence only)

| Measurement | Observed flat captures |
|---|---|
| Nearest valid line | minimum 0.316 m; mean nearest 0.335 m; worst nearest 0.432 m (start T view) |
| Farthest valid line | maximum 1.373 m radial; mean farthest 0.983 m; shortest farthest 0.531 m (finish terminates) |
| Corner first visible distance | Not measured dynamically; stationary landmark is 0.52 m ahead |
| Ground XY error | Table above; matched-frame error, not an external calibration study |
| Longest line loss | Zero line-not-found frames in each 4 s flat sample; motion/hill study remains D-07 |

The ROI caps **forward x** at 1.2 m; reported ranges are radial, including lateral geometry. A scene ending at a T limits its farthest line. These are scene-conditioned observations, not unconditional sensor limits. `design.md` remains unchanged.

### Visual files

[Mesh-derived overview](d05/mesh_overview.png). Exact capture frames with segment colours and junction leg counts; top-down independent GT vs detector.

| Pose | Camera + detections | World XY comparison |
|---|---|---|
| start_t | [overlay](d05/start_t_overlay.png) | [top down](d05/start_t_topdown.png) |
| straight | [overlay](d05/straight_overlay.png) | [top down](d05/straight_topdown.png) |
| corner | [overlay](d05/corner_overlay.png) | [top down](d05/corner_topdown.png) |
| curve | [overlay](d05/curve_overlay.png) | [top down](d05/curve_topdown.png) |
| crossing | [overlay](d05/crossing_overlay.png) | [top down](d05/crossing_topdown.png) |
| finish_t | [overlay](d05/finish_t_overlay.png) | [top down](d05/finish_t_topdown.png) |
| hill_entry | [overlay](d05/hill_entry_overlay.png) | [top down](d05/hill_entry_topdown.png) |

## Final rate measurements with the real detector

`tools/lab_env.sh check-camera-rate` exited 0. Each bag is ready before the 20 s C++ window; per-topic Hz uses its own record timestamps. Both camera publishers were observed as RELIABLE/VOLATILE.

| Variant | Images / topic interval | Camera Hz | Status Hz | Python reliable checker Hz | C++ SensorData Hz | CLI last Hz | C++-observed processing p50/p95 ms |
|---|---:|---:|---:|---:|---:|---|
| standalone | 644 / 21.210 s | 30.316 | 28.100 | 30.294 | 28.449 | 28.559 | 2.30846 / 2.89282 |
| composed | 686 / 22.605 s | 30.303 | 30.301 | 30.321 | 18.999 | 15.643 | 2.62355 / 4.4984 |

The composed C++ probe received 606 tiny detector statuses in its ~20 s window (~30.3 Hz), while its SensorData image reader remained lossy. Reliable recording/checker consumers receive the full image stream. The detector remains sustainable under the 33.3 ms/frame budget; these are throughput/latency observations, not a claim of composition speedup. Raw JSON: `log_lab/d05-camera/results.json`. A separate reliable C++ probe before the unload-lifetime-only fix received 611 images in 20.0005 s (~30.5 Hz), recording 610 in 20.127 s (~30.3 Hz), with processing p50/p95 3.53/4.30 ms (`real-reliable.json`, `d05-camera-final-final.log`). Small discovery/backlog differences explain ~30.3 vs ~30.5. The final sensor check now measures ~30 Hz with reliable QoS rather than the earlier 19–27 Hz best-effort rate.

## Final regression outcomes

**PASS for every requested command (all exits 0), with the teardown limitation below.** Consolidated exact final runs: `log_lab/d05-acceptance-final-results.json`; component-lifetime and visual-follow-up run ledgers are retained separately. Earlier development failures do not replace these final outcomes.

| Command | Exit | Wall s | Key result |
|---|---:|---:|---|
| `tools/lab_env.sh build` | 0 | 3.92 | 8 packages built, Release |
| `tools/lab_env.sh test` | 0 | 22.23 | 115 tests; 0 errors, 0 failures, 0 skipped |
| `tools/lab_env.sh check-d05` | 0 | 56.15 | six flat poses PASS; hill informational |
| `tools/lab_env.sh checkpoint-a` | 0 | 113.67 | D-01/D-02/D-03/D-04 PASS; 115 tests green |
| `tools/lab_env.sh check-gcs` | 0 | 26.47 | D-11/D-12 PASS, 26/26 |
| `tools/lab_env.sh check-bag` | 0 | 42.65 | D-13 record/replay PASS |
| `tools/lab_env.sh check-camera-rate` | 0 | 58.60 | both reliable camera recordings >30 Hz |
| `docker build -t line-follower-d05:local .` | 0 | 91.73 | production 6-package build, BUILD_TESTING=OFF |

Docker image: `sha256:3fd3bb86bbe1508adeefbb3fff6dac531231257eeb3828140e9f26a5b54b8eab`.

### Checkpoint A measured tables

Exact final command: `tools/lab_env.sh checkpoint-a`; log: `log_lab/d05-checkpoint-a-final.log`.

```text
Summary: 8 packages finished [18.2s]
Summary: 115 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A test: PASS
PASS | /camera/image_raw              | 151 messages/5s; ['sensor_msgs/msg/Image']
PASS | /camera/camera_info            | 152 messages/5s; ['sensor_msgs/msg/CameraInfo']
PASS | /imu                           | 1001 messages/5s; ['sensor_msgs/msg/Imu']
PASS | /odom                          | 250 messages/5s; ['nav_msgs/msg/Odometry']
PASS | /clock                         | 1251 messages/5s; ['rosgraph_msgs/msg/Clock']
PASS | camera dimensions and rate     | 640x480; measured 30.20 Hz wall (world target 30 Hz)
PASS | velocity bridge subscriber     | [('checkpoint_a_sensors', 'geometry_msgs/msg/Twist'), ('ros_gz_bridge', 'geometry_msgs/msg/Twist')]
PASS | camera bridge publisher QoS    | [('camera_bridge', 'VOLATILE', 'UNKNOWN')]
PASS | competition component list     | exit=0; /perception_container
PASS | no scoped bridge interfaces    | ROS graph hits=[]; bridge config scanned
PASS | provided asset SHA256          | 12 files verified; 
PASS | owned process group cleaned    | live PIDs=[]
sensors: PASS (12/12 checks)
CHECKPOINT A check-sensors: PASS
PASS | forward                        | dx=+0.0691m dy=+0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after forward             | stop=0.040s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | backward                       | dx=-0.0713m dy=-0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after backward            | stop=0.083s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | left turn                      | dx=+0.0441m dy=+0.0137m yaw=+0.5940rad; peak=0.040/0.500
PASS | stop after left turn           | stop=0.060s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | right turn                     | dx=+0.0445m dy=-0.0140m yaw=-0.6000rad; peak=0.040/0.500
PASS | stop after right turn          | stop=0.060s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate left                    | dx=-0.0000m dy=-0.0000m yaw=+0.7152rad; peak=0.000/0.600
PASS | stop after rotate left         | stop=0.044s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate right                   | dx=-0.0000m dy=+0.0000m yaw=-0.7200rad; peak=0.000/0.600
PASS | stop after rotate right        | stop=0.048s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | bounded excursion              | max from spawn odom=0.0904m
PASS | owned process group cleaned    | live PIDs=[]
motion: PASS (14/14 checks)
CHECKPOINT A check-motion: PASS
PASS | 20 second startup gate         | elapsed=20.204s; launch wall=20.844s
PASS | all pre-RUNNING commands zero  | 862 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 861 samples; displacement=0.000000m
PASS | perception pipeline drives     | odom speed=0.0274m/s
PASS | controller loss command stop   | PID=69864; zero after 0.203s; allowed=0.300s
PASS | controller loss physical stop  | stop after 0.253s; final speed=(1.1926223897340549e-14, 8.915390686640387e-14)
PASS | safety explains stop           | ['', 'command candidate stale']
PASS | owned process group cleaned    | live PIDs=[]
start-stop: PASS (8/8 checks)
CHECKPOINT A check-start-stop: PASS
PASS | 20 second startup gate         | elapsed=20.205s; launch wall=20.845s
PASS | all pre-RUNNING commands zero  | 862 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 861 samples; displacement=0.000000m
PASS | perception pipeline drives     | odom speed=0.0274m/s
PASS | safety_guard exit leaves zero  | PID=70086; post-exit messages=6; last_zero=True
PASS | safety_guard exit physical stop | stop after 0.056s; final speed=(1.0760706561918632e-15, 7.425084912962575e-14)
PASS | owned process group cleaned    | live PIDs=[]
guard-exit: PASS (7/7 checks)
CHECKPOINT A check-guard-exit: PASS
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
```

### GCS measured table

Exact final command: `tools/lab_env.sh check-gcs`; log: `log_lab/d05-check-gcs-final.log`.

```text
PASS | START before READY rejected    | rejected: mission not READY
PASS | wrong run rejected             | run_id mismatch
PASS | bag START missing recorder     | timeout: service unavailable or response deadline exceeded
PASS | bag STOP missing recorder      | timeout: service unavailable or response deadline exceeded
PASS | GCS snapshot rate              | 6 samples; 2.000 Hz
PASS | snapshot relay state equality  | 6 matched stamps; max relay=0.0035s
PASS | odom geometry and provenance   | graph=1 path=59 sources=[0]
PASS | link loss visible and isolated | LOST after 2.044s; all five rover nodes alive; mission publishes
PASS | link recovers                  | OK
PASS | START READY manual accepted    | start requested
PASS | duplicate returns original     | accepted=True; cached state=2; same response=True
PASS | START RUNNING rejected         | rejected: mission not READY
PASS | live RUNNING reaches UI feed   | GCS state=3
PASS | RUNNING display within period  | rover→GCS state delay=0.3012s (2 Hz + 50ms transport margin)
PASS | real rover moves               | odom speed=0.1000m/s
PASS | UI exit does not stop rover    | UI PID=70311; rover alive, mission heartbeat, speed=0.1000
PASS | KILL active accepted           | accepted: kill requested (software stop, not a physical E-STOP)
PASS | KILL E_STOP then STOPPED       | states=[0, 1, 2, 3, 7, 8]
PASS | KILL final velocity zero       | cmd=0.0; odom=0.000000
PASS | KILL inactive rejected         | rejected: mission already inactive
PASS | timeline retains short states  | GCS transition events=[1, 2, 3, 7, 8]
PASS | live Qt map/camera render      | last render=('36', '83', '3')
PASS | live UI reception              | snapshots/camera/elapsed/Hz=('36', '83', '19.97', '1.80', '4.16')
PASS | GCS processes healthy          | []
PASS | stopped sources marked stale   | stale_sources=['candidates', 'observed_graph', 'path_status', 'perception_status', 'safety_status', 'selected_path']; odom remains fresh
PASS | owned process group cleaned    | live PIDs=[]
GCS D-11/D-12: PASS (26/26)
```

### Bag/replay measured table

Exact final command: `tools/lab_env.sh check-bag`; log: `log_lab/d05-check-bag-final.log`.

```text
PASS | STOP while idle rejected       | not recording
PASS | recorder wrong run rejected    | run_id mismatch
PASS | record START accepted          | /ws/bags/check-stage2/eb280a3f/20261119T072955_352bbfb7
PASS | duplicate START original reply | /ws/bags/check-stage2/eb280a3f/20261119T072955_352bbfb7
PASS | different command ID rejected  | request_id reused with different command
PASS | second START rejected          | already recording
PASS | recording status active        | state=RECORDING
PASS | recorded KILL accepted         | accepted: kill requested (software stop, not a physical E-STOP)
PASS | recorded rover STOPPED         | state=8
PASS | record STOP closes cleanly     | stop requested; close=0.266s
PASS | duplicate STOP original reply  | stop requested
PASS | second STOP rejected           | not recording
PASS | recording status STOPPED       | STOPPED
PASS | bag metadata contract          | {"closed_cleanly": true, "code_config_sha256": "42b3ecddf967f2670170ab299c54b70fdf3dc7317dc7e858419ffe364239aeb0", "environment": {"compression": "zstd_fast", "gz_partition": "bag_record_72218", "hostname": "9704990b1686", "platform": "Linux-7.0.0-42-generic-x86_64-with-glibc2.39", "ros_distro": "jazzy", "ros_domain_id": "84", "storage": "mcap"}, "params_file": "/ws/install_lab/line_follower_bringup/share/line_follower_bringup/config/competition.yaml", "recorder_exit_code": 0, "recorder_requests": [{"accepted": true, "command": 0, "reason": "/ws/bags/check-stage2/eb280a3f/20261119T072955_352bbfb7", "request_id": "e16ff113-050b-4507-a59c-c47abcd97fa2", "sent_stamp_ns": 0}, {"accepted": false, "command": 0, "reason": "already recording", "request_id": "c43e4aef-c989-4001-9a23-8e0e1c89ae37", "sent_stamp_ns": 12000000}, {"accepted": true, "command": 1, "reason": "stop requested", "request_id": "9763f455-5f97-45a4-abd0-094fe6dd9273", "sent_stamp_ns": 18632000000}, {"accepted": false, "command": 1, "reason": "not recording", "request_id": "cd747dd9-802e-4a82-a907-3dcfa28b80ec", "sent_stamp_ns": 18904000000}], "run_id": "run_20261119_072953_879240302447397", "schema_version": 1, "start": {"ros_ns": 8000000, "utc": "2026-11-19T07:29:55.904142+00:00", "wall_monotonic": 879242.994319121}, "stop": {"ros_ns": 18636000000, "utc": "2026-11-19T07:30:15.192433+00:00", "wall_monotonic": 879262.282615059}, "stop_reason": "stop requested", "topics": ["/camera/image_raw", "/camera/camera_info", "/clock", "/imu", "/odom", "/localization/estimate", "/perception/line", "/perception/status", "/path/observed_graph", "/path/local", "/path/candidates", "/path/status", "/control/cmd_vel_candidate", "/cmd_vel", "/mission/state", "/safety/status", "/perception/obstacles", "/mission/gcs_snapshot", "/gcs/command_events", "/gcs/recording_status"], "use_sim_time": true}
PASS | expected topics and counts     | missing=[]; counts={"/camera/camera_info": 541, "/camera/image_raw": 540, "/clock": 4463, "/cmd_vel": 891, "/control/cmd_vel_candidate": 541, "/gcs/command_events": 1, "/gcs/recording_status": 86, "/imu": 3573, "/localization/estimate": 892, "/mission/gcs_snapshot": 36, "/mission/state": 189, "/odom": 893, "/path/candidates": 541, "/path/local": 542, "/path/observed_graph": 542, "/path/status": 541, "/perception/line": 541, "/perception/status": 542, "/safety/status": 891}
PASS | full mission event sequence    | ['STARTUP_WAIT', 'CHECKING', 'READY', 'RUNNING', 'E_STOP', 'STOPPED']
PASS | KILL command event             | 12 total events
EVENT 0.000s perception: healthy=True fallback=False 
EVENT 0.020s mission: STARTUP_WAIT
EVENT 0.044s mode: VISION_TRACKING
EVENT 0.056s safety: drive not enabled
EVENT 16.156s mission: CHECKING
EVENT 16.256s mission: READY
EVENT 16.356s mission: RUNNING
EVENT 16.376s safety: command approved
EVENT 16.756s mission: E_STOP
EVENT 16.756s command: kill: accepted kill requested (software stop, not a physical E-STOP)
EVENT 16.776s safety: drive not enabled
EVENT 16.856s mission: STOPPED
PASS | CLI uses shared extractor      | JSON=/ws/bags/check-stage2/eb280a3f/analysis.json; exit=0; stderr=
PASS | bounded recording artifact     | bag=/ws/bags/check-stage2/eb280a3f/20261119T072955_352bbfb7; total=11.30 MiB; duration=17.860s
PASS | UI bag RUNNING scrub | t=16.657s; map observed/planned; camera=640x480; commands disabled
PASS | UI bag end scrub | state=STOPPED; events=12; same shared extractor
PASS | UI LIVE return | replay ended; seek disabled
PASS | real bag offscreen scrub       | exit=0; stderr=QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-ubuntu'
PASS | owned process group cleaned    | live PIDs=[]
PASS | replay never publishes cmd_vel | max publishers=0; received=0
PASS | replay command redirected      | /replay/cmd_vel_blocked count=575; nonzero=422
PASS | perception/path regenerated    | {"/path/local": 459, "/path/status": 458, "/perception/line": 415, "/perception/status": 415}
PASS | replay excludes Gazebo bridge  | nodes=['checkpoint_a_bag_replay', 'launch_ros_72534', 'line_detector', 'mission_manager', 'motion_controller', 'path_planner', 'perception_container', 'safety_guard']
PASS | owned process group cleaned    | live PIDs=[]
D-13 RECORD/REPLAY: PASS; artifacts=/ws/bags/check-stage2/eb280a3f
```

## Remaining limitation and cleanup

The active detector-unload crash was reproduced and fixed; the final 26/26 GCS check confirms actual unload and stale-source detection. A **separate whole-launch SIGINT teardown still logs `component_container_mt` exit code -11** in some composed runs (including the D-05 capture launch `/tmp/checkpoint-d05-5pdyx5hp/launch.log`). Its remaining root cause has not been isolated. This is reported explicitly: the acceptance scripts verify empty owned process groups, but do not assert a normal exit code for every shutdown child. The requested checker commands themselves all return 0. No steady-state crash occurred in the final runs. Full shutdown diagnostics are in `log_lab/d05-teardown-diagnostics.json`; this issue is not presented as resolved.

Final process inspection is saved in `log_lab/d05-final-processes.log`: only container init/sleep and the inspection command remain; no Gazebo/launch/GCS/recorder/player/probe process. Camera-rate bags use MCAP zstd and only the two owned fixed directories are retained (bounded by the ~20 s windows); D-13 retains one short marked check recording. No other container or host project process was signalled. No provided world/model/hash file, original parent asset or design checkbox changed.

## Files added/changed

- `README.md`
- `docs/test_plan.md`
- `docs/d05_evidence.md`
- `src/line_follower_perception/include/line_follower_perception/line_detector_backend.hpp`
- `src/line_follower_perception/src/line_detector_backend.cpp`
- `src/line_follower_perception/src/line_detector_node.cpp`
- `src/line_follower_perception/test/test_line_detector_backend.cpp`
- `src/line_follower_navigation/src/local_path_mapper.cpp`
- `src/line_follower_navigation/src/trajectory_planner.cpp`
- `src/line_follower_navigation/test/test_trajectory_planner.cpp`
- `src/line_follower_bringup/config/camera_bridge.yaml`
- `src/line_follower_bringup/config/competition.yaml`
- `src/line_follower_tests/CMakeLists.txt`
- `src/line_follower_tests/package.xml`
- `src/line_follower_tests/test/test_component_factories.cpp`
- `src/line_follower_tests/test/test_d05_groundtruth.py`
- `src/line_follower_tests/test/test_pipeline_launch.py`
- `src/line_follower_tests/test/test_gcs_launch.py`
- `src/line_follower_tests/test/test_static_policy.py`
- `tools/lab_env.sh`
- `tools/checkpoint_a.py`
- `tools/check_d05.py`
- `tools/d05_capture.launch.py`
- `tools/d05_groundtruth.py`
- `tools/check_camera_rate.py`
- `tools/camera_probe/CMakeLists.txt`
- `tools/camera_probe/COLCON_IGNORE`
- `tools/camera_probe/main.cpp`
- `docs/d05/corner_overlay.png`
- `docs/d05/corner_topdown.png`
- `docs/d05/crossing_overlay.png`
- `docs/d05/crossing_topdown.png`
- `docs/d05/curve_overlay.png`
- `docs/d05/curve_topdown.png`
- `docs/d05/finish_t_overlay.png`
- `docs/d05/finish_t_topdown.png`
- `docs/d05/hill_entry_overlay.png`
- `docs/d05/hill_entry_topdown.png`
- `docs/d05/measurements.json`
- `docs/d05/mesh_overview.png`
- `docs/d05/start_t_overlay.png`
- `docs/d05/start_t_topdown.png`
- `docs/d05/straight_overlay.png`
- `docs/d05/straight_topdown.png`

Raw logs and retained bags are ignored artifacts, not competition package dependencies.
