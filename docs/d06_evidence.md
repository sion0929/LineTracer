# D-06 — capture-time odom mapping, bounded memory and moving Gazebo evidence

Date: 2026-11-19. ROS 2 Jazzy / Gazebo Harmonic inside `line_follower_lab`.
Final command results, raw tables and artifact links are appended below after the final runs.
Only the final exits determine acceptance; earlier failures and rejected hypotheses are recorded.

## Commands and isolation

```bash
cd '/home/sion/Line_Tracer /line_follower_ws'
tools/lab_env.sh build
tools/lab_env.sh test
tools/lab_env.sh check-teardown
tools/lab_env.sh check-d06
tools/lab_env.sh check-d05
tools/lab_env.sh checkpoint-a
tools/lab_env.sh check-gcs
tools/lab_env.sh check-bag
tools/lab_env.sh check-camera-rate
docker build -t line-follower-d06:local .
```

No host ROS commands or unrelated containers/process groups were used. Teardown uses domain 94,
moving D-06 domain 95, unit investigation domain 96; every Gazebo run uses a unique partition. Existing
checks retain their dedicated domains. Original worlds/models/SHA256SUMS and parent originals remain
unchanged, and no design checkbox is edited. The unchanged production Dockerfile still builds only
six rover packages up to bringup with Release / BUILD_TESTING=OFF, with no GCS/Qt/dev-tool dependency.

## Part 0 — teardown isolation, backtrace and mitigation

The pre-fix repeated full competition check failed **0/2 clean**, both `component_container_mt` -11
(`log_lab/d06/teardown-red.log`, `baseline/{01,02}.log`). Isolation using the same immutable Gazebo
world found **camera-only 0/2 clean**, **app-only (external image bridge) 2/2 clean** (`camera-only.log`,
`apps-only.log`). Thus our detector/planner alone were not required for the shutdown crash.

GDB was available on the host and copied with its dependencies into ignored workspace artifacts so
it could run inside the lab without changing the image. The inferior used the original container
library path. The first debug probe failed its PID locator because GDB used a separate inferior
process group; owned-launch cleanup still delivered SIGINT and captured a backtrace. This probe is
not counted as a passing teardown test. `gdb-backtrace.log` shows worker PCs at unmapped addresses,
while the main thread is in `eprosima::fastdds::dds::DomainParticipantFactory` static destruction /
process exit. Python pretty-printer support was unavailable, and a precise upstream source line was
not obtained. Evidence supports a bridge/shared-library shutdown-lifetime problem; the exact internal
transport function is not claimed.

A container-only `LD_PRELOAD` of the SDK `libros_gz_bridge.so` holds that library and its dependencies
mapped until process exit, after global shutdown work. The same three-component capture graph then
exited normally **3/3** (`teardown-preload.log`). The production launch resolves the library through
the installed package prefix, enables this only when loading the camera bridge component, preserves
existing preload entries, and leaves the four-thread executor, intra-process options, topics, QoS
and camera stamps unchanged. Replay loads only app components and gets no bridge preload. The D-05
capture launch uses the same mitigation.

Two launch-configuration cases were added: the camera-enabled case was red before the conditional preload, while the no-camera case already passed (`lifetime-config-red.log`).
The first full ten-run competition check passed **10/10** (`teardown-final.log`); the final rerun after
the full D-06 implementation is tabulated below. This fixes the prior D-05 teardown limitation through
launch configuration; it does not patch system SDK libraries. No custom shutdown delay or ignored
child crash is used: the checker requires a normal container exit and an empty owned group.

## Implementation and contracts

**Capture-time pose.** PoseEstimator keeps up to 600 ordered odom samples. It linearly interpolates
XY and the shortest wrapped yaw between bracketing samples, rejects requests outside history and
older than the allowed capture age, and never extrapolates from the latest pose. The node briefly
queues at most three observations until the next odom sample brackets capture time. Old, reset-era
or unsupported captures produce a stop; the wall watchdog remains independent of input arrival.
Odometry and IMU must both be fresh by ROS source stamps and steady-clock receipt deadlines for health/memory use; invalid IMU orientation is rejected.
Localization is raw odometry pose plus IMU pitch, with no EKF or IMU-yaw fusion.

**Observed graph.** LocalPathMapper transforms directly observed base-frame segments at capture
pose into `odom`, preserves capture stamps, connectivity/junctions and uncertainty, and rejects an
entire segment containing UNKNOWN/memory/nonfinite points rather than bridging a missing section.
Invalid projection produces no confirmed graph. Canonical IDs come from geometry association in the
memory, so current observed and retained graphs can refer to the same tracked segments. The published
`/path/observed_graph` contains raw current observations only, never averaged memory geometry.

**Memory.** Overlap, distance and local tangent compatibility associate incoming segments; an old track
is used at most once per image to preserve distinct crossing legs. Incoming detector connections,
rather than adjacency alone, establish edges. Repeated nearby points merge; observed extensions add
only actual observed coordinates. Connected endpoint pairs share one anchor and keep separate legs.
Geometry fusion is retained-memory data, not relabelled as a raw observation. Default limits are
1,200 points, 32 segments, 10 s retention, 2 m local radius, passed threshold -0.08 m and uncertainty
limit 0.07 m. Bounded eviction handles excess geometry; interior expiry drops a segment conservatively
instead of connecting across an unobserved gap. Some far-field fragments hit the segment cap in the
moving check; the point count remains bounded and repeated-view unit tests verify actual deduplication.
This is a local temporary map, not a global SLAM topology solution.

Uncertainty grows by 0.004 m/s, 0.025 m per travelled metre and 0.003 m per absolute radian between
updates; confidence decays with age/travel. Re-observation refreshes matched metadata; point expiry
remains based on original/last actual capture. New run IDs, terminal transitions and fault recovery
clear memory. The core reset is also exposed as `/path/reset_memory` (`std_srvs/Trigger`), used by the
dev harness after teleporting. Reset clears pending pre-reset captures, publishes empty graphs and a
stopped path. Clock reversal resets history/memory. No official track coordinate, file or layout is
read by runtime navigation code.

**Local path.** The nearest forward retained chain follows only recorded unambiguous endpoint
connections; D-08 route/branch ranking is not added. Its coordinates use the **current base pose**.
`LocalPath.header.stamp` is therefore the **odometry basis stamp**, while each point’s `capture_stamp`
keeps its own actual camera time. This replaces the prior placeholder assumption that every local-path
header must equal the image stamp; carrying that assumption into a moving base frame would misstate
the coordinate basis. The node-only test now checks capture stamps through line/observed graph and
per path point, plus odom-basis stamps on local paths. No provenance assertion was removed.

Retained points are MEMORY_UPDATED. A matching current raw graph point supplies exact OBSERVED
coordinates/metadata; fused coordinates are not labelled direct observations. UNKNOWN never enters
output. Point expiry caps path validity, and mixed memory paths have a conservative 0.06 m/s speed
cap; a memory target reports MEMORY_TRACKING. Full line loss, projection invalidity or sensor
staleness stops output rather than enabling D-07 hill/full-loss fallback. Controller target/speed/
validity fields and the safety graph remain intact.

`/path/memory` (existing ObservedGraph type, `odom`) exposes bounded retained geometry and is included
in the single-source bag topic list. The existing mission_manager snapshot transports mixed-source
local paths and robot pose to the unchanged GCS UI; cyan memory rendering already existed. Replay
also verifies this newly regenerated topic. There is no new competition dependency on a dev package.

## Test-first work and earlier failures

- New C++ capture/memory tests were written before the new APIs; the initial navigation build failed
  on missing `update`, `set_run_id`, `path_to_base` and capture-age interpolation API (`navigation-red.log`).
  A subsequent compile attempt exposed an iterator declaration and a test using nonexistent C++
  `Time.to_msg`; those were corrected before tests ran.
- The first moving run failed memory growth during rotation and localization comparison. Association
  gates and point merging were corrected; pre-placement observations are now cleared explicitly.
  The checker’s odom reference is reliable, preserving exact timestamp matches; it does not loosen
  the 2 mm / 5 mrad localization bound. Transverse crossing legs are excluded from the same-straight
  normal-spread statistic by direction, rather than being counted as straight-line smearing.
- At 0.25 rad/s, wheel odom and IMU yaw disagreed by **0.04515 rad**; far-field points fanned in the
  odom-to-world comparison and spread was ~0.030 m (`rotate-diagnostic.log`). A 0.1 rad/s run retained
  ±0.5 rad excursion but reduced disagreement below 0.01 rad and passed the original 0.015 m spread
  limit (`rotate-slow.log`). No yaw fusion or safety-threshold relaxation was added. This is a measured
  speed/accuracy limitation of the odom baseline, not evidence for arbitrary-speed map stability.
- A suspected moving-window advection bug was tested and **not reproduced**: that new test passed
  the existing implementation, and no advection fix is claimed. It remains a regression test.
- The first corner-angle check used just two neighboring samples, including duplicate shared-vertex
  samples and sub-centimetre quantization, and reported ~58–62 degrees despite millimetre vertex error.
  The final estimator uses leg support at least 0.06 m away; angle limits remain 75–105 degrees and
  vertex error was **tightened** from 0.030 to 0.015 m. Shared-endpoint anchoring was added as a graph
  invariant; the noisy synthetic case already passed before that addition, so no false red result is
  claimed for that case. The final Gazebo corner table reports the measured angle and vertex error.
- A final frozen-ROS-clock unit test was red (`frozen-clock-red.log`): source timestamp age alone kept stopped sensors healthy. PoseEstimator now also checks steady-clock receipt age for both sensors, and the complete final regression was rerun after this fix.
- One intermediate diagnostic script had a comma typo and exited before launching; it was corrected.

Ten C++ mapping/memory cases cover wrapped-yaw interpolation and outside/old rejection; both-sensor
freshness; known odom transform and direct-only graph; 500 repeated views; time/travel uncertainty,
expiry and passed geometry; run/fault reset and capacity; current-base blind corner with sharp angle;
a moving observation window; a shared vertex under repeated noisy views; and stale-sensor detection while ROS time is frozen. Existing navigation,
mission, safety, component unload, fake-pipeline and GCS tests remain in the full test count below.

## Moving harness and measurement method

`tools/d06_capture.launch.py` starts headless Gazebo + bridges + the perception container and
mission_manager solely for the 2 Hz snapshot. No controller/safety_guard is loaded. The sole velocity
publisher is verified as the harness node. Every exit sends repeated zero Twist before owned-group
cleanup. Three independent launches use mesh-derived flat placements, settle, record the odom/world
rigid transform, explicitly reset pre-teleport memory, and collect moving samples.

Straight: ~0.7 m at 0.07 m/s. Corner: ~0.39 m at 0.06 m/s, stopping with the known vertex inside the
~0.316 m camera blind zone. Rotation: +0.5, -0.5 and back to 0 rad at 0.1 rad/s; no translation command.
Graph samples are retained at roughly 6 Hz; localization/snapshot samples retain their own timestamps.
World XY errors use the independent STL strip oracle and the initial odom/world transform. This
includes physical wheel-odom drift; it does not use simulator ground truth to correct runtime pose.

Initial error gates remain observed median/p95/max <0.030/0.055/0.100 m and memory p95/max
<0.055/0.100 m; same-straight normal spread <0.015 m. Point/segment caps and end density, frames and
sources, timestamp-matched localization, snapshot pose, commanded excursion, blind-memory coverage,
corner vertex/outgoing leg/angle, and final zero/physical stop all gate the command. These are flat
slow-motion tests, not full-track autonomous control, hill fallback, EKF or branch-decision validation.

## Final D-06 moving results

`tools/lab_env.sh check-d06` exited **0**, all three scenarios PASS. Full exact rows: `log_lab/d06/check-d06-final.log`; structured data: [`d06/measurements.json`](d06/measurements.json). Geometry errors include all retained samples in the moving interval.

| Scenario | Observed median / p95 / max (m) | Memory median / p95 / max (m) | Peak / end points | Peak segments | Normal spread (m) | Travel / yaw |
|---|---|---|---|---:|---:|---|
| straight | 0.00005 / 0.01257 / 0.03005 | 0.00346 / 0.01247 / 0.03353 | 265 / 252 | 32 | 0.00000 | 0.700 m |
| corner | 0.00881 / 0.01943 / 0.02400 | 0.00905 / 0.01703 / 0.02400 | 137 / 134 | 4 | 0.00000 | 0.389 m |
| rotate | 0.00483 / 0.01357 / 0.03312 | 0.00550 / 0.01451 / 0.02974 | 380 / 176 | 32 | 0.00935 | 0.503 rad; 3.41e-12 m |

| Scenario | Localization max XY / yaw error | Snapshot max XY / yaw error | UNKNOWN points | Local memory points (sample sum) |
|---|---|---|---:|---:|
| straight | 0.000000 m / 0 rad | 0.000000 m / 0 rad | 0 | 857 |
| corner | 0.000000 m / 0 rad | 0.001200 m / 1.32e-13 rad | 0 | 899 |
| rotate | 0.000000 m / 5.55e-17 rad | 0.000000 m / 0 rad | 0 | 127 |

Straight local MEMORY_UPDATED points cover **0.010–0.251 m** ahead inside the blind zone. Corner is **0.211 m** ahead, vertex memory error **0.00180 m**, supported leg angle **84.04 degrees**, with the outgoing leg retained. Slow-rotation odom/IMU-yaw discrepancy peaks at **0.00892 rad**. Final zero command and physical stop pass in every scenario.

### Plot files

- [Straight ground truth / direct / retained / local-memory / robot trajectory](d06/straight_topdown.png)
- [Blind corner and retained outgoing leg](d06/corner_topdown.png)
- [In-place rotation and heading arrows](d06/rotate_topdown.png)

GT is grey, direct observed green, retained memory cyan, local memory blue, robot trajectory/headings red. No runtime code consumes this ground truth.

### Exact final moving PASS table

```text
PASS | dev graph command ownership    | no controller/guard; publishers=['checkpoint_a_d05']; scenario=straight
PASS | observed/memory ground truth   | {"displacement_m": 0.7002079999987909, "imu_odom_yaw_max_difference": 2.482751835080569e-05, "localization_max_error": [0.0, 0.0], "memory_end_points": 252, "memory_error_m": [0.0034628252105052063, 0.012466025708078098, 0.03352826055246697], "memory_peak_points": 265, "memory_peak_segments": 32, "memory_samples": 65, "memory_source_points": 857, "normal_spread_max_m": 6.400088169356562e-14, "observed_error_m": [5.136518411585911e-05, 0.012568138281051055, 0.03005162092130268], "observed_samples": 65, "odom_to_world": [0.6900000095371073, 2.309999942779541, -8.218975208566472e-13], "path_samples": 65, "scenario": "straight", "snapshot_max_error": [0.0, 0.0], "spawn_world": [0.6900000095367431, 2.309999942779541, 0.01, 0.0], "unknown_points": 0}
PASS | bounded deduplicated memory    | peak/end points=265/252; segments=32
PASS | frame and source contract      | unknown=0; observed/memory=odom; local=base_footprint
PASS | odom localization and snapshot | loc max=[0.0, 0.0]; snapshot max=[0.0, 0.0]
PASS | straight blind zone uses memory | MEMORY_UPDATED near range=[0.010037051140315056, 0.25065833322346043]
PASS | forward distance and stable map | displacement=0.700m; normal spread=0.00000m
PASS | stop leaves zero               | last_zero=True; speed=(1.3877787807814457e-14, 9.670932270253976e-14)
PASS | owned process group cleaned    | live PIDs=[]
PASS | dev graph command ownership    | no controller/guard; publishers=['checkpoint_a_d05']; scenario=corner
PASS | observed/memory ground truth   | {"displacement_m": 0.3888000000001313, "imu_odom_yaw_max_difference": 2.3899957426465335e-05, "localization_max_error": [0.0, 0.0], "memory_end_points": 134, "memory_error_m": [0.009049461130431702, 0.017031623517030064, 0.023995042547285905], "memory_peak_points": 137, "memory_peak_segments": 4, "memory_samples": 42, "memory_source_points": 899, "normal_spread_max_m": 0.0, "observed_error_m": [0.008806771216756992, 0.019429746858183217, 0.0239999790203429], "observed_samples": 42, "odom_to_world": [0.9900000095367432, 1.0499999761583456, 1.570796326796283], "path_samples": 42, "scenario": "corner", "snapshot_max_error": [0.0012000000000023103, 1.3229835345707456e-13], "spawn_world": [0.9900000095367432, 1.049999976158142, 0.01, 1.5707963267948966], "unknown_points": 0}
PASS | bounded deduplicated memory    | peak/end points=137/134; segments=4
PASS | frame and source contract      | unknown=0; observed/memory=odom; local=base_footprint
PASS | odom localization and snapshot | loc max=[0.0, 0.0]; snapshot max=[0.0012000000000023103, 1.3229835345707456e-13]
PASS | blind corner remains sharp memory | vertex error=0.0018049730581066736; outgoing=True; angle=84.03685986181785; corner ahead=0.211m < nearest view~0.316m
PASS | stop leaves zero               | last_zero=True; speed=(1.214306433183765e-13, 2.2726690835096846e-13)
PASS | owned process group cleaned    | live PIDs=[]
PASS | dev graph command ownership    | no controller/guard; publishers=['checkpoint_a_d05']; scenario=rotate
PASS | observed/memory ground truth   | {"displacement_m": 3.405851730122913e-12, "imu_odom_yaw_max_difference": 0.008918353475525118, "localization_max_error": [0.0, 5.551115123125783e-17], "memory_end_points": 176, "memory_error_m": [0.005496417289989278, 0.01450724396636005, 0.029737940988163295], "memory_peak_points": 380, "memory_peak_segments": 32, "memory_samples": 124, "memory_source_points": 127, "normal_spread_max_m": 0.009346567415555368, "observed_error_m": [0.004826570019960563, 0.013566412639121018, 0.033123261328437556], "observed_samples": 124, "odom_to_world": [0.9900000095369457, 2.309999942779541, -7.722806150555709e-13], "path_samples": 125, "scenario": "rotate", "snapshot_max_error": [0.0, 0.0], "spawn_world": [0.9900000095367432, 2.309999942779541, 0.01, 0.0], "unknown_points": 0}
PASS | bounded deduplicated memory    | peak/end points=380/176; segments=32
PASS | frame and source contract      | unknown=0; observed/memory=odom; local=base_footprint
PASS | odom localization and snapshot | loc max=[0.0, 5.551115123125783e-17]; snapshot max=[0.0, 0.0]
PASS | rotation does not smear map    | peak yaw=0.503rad; translation=0.0000m; spread=0.00935m
PASS | stop leaves zero               | last_zero=True; speed=(1.627346803319274e-13, 4.847649243162757e-13)
PASS | owned process group cleaned    | live PIDs=[]
D-06: PASS
```

## Final teardown results

The requested final ten repetitions are **10/10 normal component-container exits**, all with a running camera and empty owned groups; the earlier two ten-run verification rounds also passed (30/30 total across the staged implementation). Only the last ten are used for the exact final-source acceptance. SDK Gazebo shutdown may still print its usual SIGINT/server-adapter shutdown diagnostics; the checker explicitly requires normal component-container exit rather than ignoring a crash.

```text
PASS | teardown 1/10 | {"iteration": 1, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/01.log"}
PASS | teardown 2/10 | {"iteration": 2, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/02.log"}
PASS | teardown 3/10 | {"iteration": 3, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/03.log"}
PASS | teardown 4/10 | {"iteration": 4, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/04.log"}
PASS | teardown 5/10 | {"iteration": 5, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/05.log"}
PASS | teardown 6/10 | {"iteration": 6, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/06.log"}
PASS | teardown 7/10 | {"iteration": 7, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/07.log"}
PASS | teardown 8/10 | {"iteration": 8, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/08.log"}
PASS | teardown 9/10 | {"iteration": 9, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/09.log"}
PASS | teardown 10/10 | {"iteration": 10, "pass": true, "camera_ready": true, "normal_exit": true, "exit_errors": [], "log": "/ws/log_lab/d06/teardown/10.log"}
TEARDOWN: PASS (10/10 clean)
```

Across all 23 simulator/replay launch logs from the final runtime commands, **zero component-container exit errors** were found (`log_lab/d06/final-teardown-audit.json`). The D-05 SIGSEGV limitation is resolved by the conditional library-lifetime mitigation in this environment; no patched SDK source is claimed.

## Final regression table

**PASS: every final command exited 0.** Exact commands/statuses/timings: `log_lab/d06/acceptance-final-results.json`; earlier run ledgers remain under `log_lab/d06/`.

| Command | Exit | Wall s | Result |
|---|---:|---:|---|
| `tools/lab_env.sh build` | 0 | 22.97 | 8 packages, Release |
| `tools/lab_env.sh test` | 0 | 18.91 | 128 tests, 0 failures, 0 errors, 0 skipped |
| `tools/lab_env.sh check-teardown` | 0 | 51.39 | 10/10 clean component exits |
| `tools/lab_env.sh check-d06` | 0 | 71.43 | three moving scenarios PASS |
| `tools/lab_env.sh check-d05` | 0 | 52.74 | six flat poses PASS; hill informational |
| `tools/lab_env.sh checkpoint-a` | 0 | 114.44 | D-01 … D-04 PASS, unchanged safety limits |
| `tools/lab_env.sh check-gcs` | 0 | 26.35 | 26/26 communication/UI checks |
| `tools/lab_env.sh check-bag` | 0 | 37.26 | record, event analysis, UI scrub and replay PASS |
| `tools/lab_env.sh check-camera-rate` | 0 | 66.32 | external reliable camera recorders PASS |
| `docker build -t line-follower-d06:local .` | 0 | 49.54 | 6 production packages; BUILD_TESTING=OFF |

Production image: `sha256:8fc3021f863741e8ef68d6b65aa3e18b0d932bef9aa5125ec8db18a90991b0d9`.

### Safety/motion regression

Command: `tools/lab_env.sh checkpoint-a`; exact final rows:

```text
Summary: 8 packages finished [19.2s]
Summary: 128 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A test: PASS
PASS | /camera/image_raw              | 151 messages/5s; ['sensor_msgs/msg/Image']
PASS | /camera/camera_info            | 152 messages/5s; ['sensor_msgs/msg/CameraInfo']
PASS | /imu                           | 1001 messages/5s; ['sensor_msgs/msg/Imu']
PASS | /odom                          | 250 messages/5s; ['nav_msgs/msg/Odometry']
PASS | /clock                         | 1252 messages/5s; ['rosgraph_msgs/msg/Clock']
PASS | camera dimensions and rate     | 640x480; measured 30.29 Hz wall (world target 30 Hz)
PASS | velocity bridge subscriber     | [('checkpoint_a_sensors', 'geometry_msgs/msg/Twist'), ('ros_gz_bridge', 'geometry_msgs/msg/Twist')]
PASS | camera bridge publisher QoS    | [('camera_bridge', 'VOLATILE', 'UNKNOWN')]
PASS | competition component list     | exit=0; /perception_container
PASS | no scoped bridge interfaces    | ROS graph hits=[]; bridge config scanned
PASS | provided asset SHA256          | 12 files verified; 
PASS | owned process group cleaned    | live PIDs=[]
sensors: PASS (12/12 checks)
CHECKPOINT A check-sensors: PASS
PASS | forward                        | dx=+0.0696m dy=+0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after forward             | stop=0.040s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | backward                       | dx=-0.0718m dy=+0.0000m yaw=-0.0000rad; peak=0.060/0.000
PASS | stop after backward            | stop=0.077s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | left turn                      | dx=+0.0446m dy=+0.0141m yaw=+0.6020rad; peak=0.040/0.500
PASS | stop after left turn           | stop=0.048s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | right turn                     | dx=+0.0439m dy=-0.0136m yaw=-0.5920rad; peak=0.040/0.500
PASS | stop after right turn          | stop=0.060s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate left                    | dx=-0.0000m dy=-0.0000m yaw=+0.7128rad; peak=0.000/0.600
PASS | stop after rotate left         | stop=0.060s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate right                   | dx=-0.0000m dy=+0.0000m yaw=-0.7200rad; peak=0.000/0.600
PASS | stop after rotate right        | stop=0.052s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | bounded excursion              | max from spawn odom=0.0917m
PASS | owned process group cleaned    | live PIDs=[]
motion: PASS (14/14 checks)
CHECKPOINT A check-motion: PASS
PASS | 20 second startup gate         | elapsed=20.204s; launch wall=20.815s
PASS | all pre-RUNNING commands zero  | 1005 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 860 samples; displacement=0.000000m
PASS | perception pipeline drives     | odom speed=0.0274m/s
PASS | controller loss command stop   | PID=95826; zero after 0.201s; allowed=0.300s
PASS | controller loss physical stop  | stop after 0.253s; final speed=(1.1449174941446927e-14, 9.247828983430367e-14)
PASS | safety explains stop           | ['', 'command candidate stale']
PASS | owned process group cleaned    | live PIDs=[]
start-stop: PASS (8/8 checks)
CHECKPOINT A check-start-stop: PASS
PASS | 20 second startup gate         | elapsed=20.204s; launch wall=20.819s
PASS | all pre-RUNNING commands zero  | 1006 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 847 samples; displacement=0.000000m
PASS | perception pipeline drives     | odom speed=0.0274m/s
PASS | safety_guard exit leaves zero  | PID=96048; post-exit messages=6; last_zero=True
PASS | safety_guard exit physical stop | stop after 0.056s; final speed=(1.0814916670542907e-15, 7.421307205044506e-14)
PASS | owned process group cleaned    | live PIDs=[]
guard-exit: PASS (7/7 checks)
CHECKPOINT A check-guard-exit: PASS
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
```

### GCS live regression

Command: `tools/lab_env.sh check-gcs`; exact final rows:

```text
PASS | START before READY rejected    | rejected: mission not READY
PASS | wrong run rejected             | run_id mismatch
PASS | bag START missing recorder     | timeout: service unavailable or response deadline exceeded
PASS | bag STOP missing recorder      | timeout: service unavailable or response deadline exceeded
PASS | GCS snapshot rate              | 6 samples; 2.013 Hz
PASS | snapshot relay state equality  | 6 matched stamps; max relay=0.0030s
PASS | odom geometry and provenance   | graph=1 path=59 sources=[0]
PASS | link loss visible and isolated | LOST after 2.021s; all five rover nodes alive; mission publishes
PASS | link recovers                  | OK
PASS | START READY manual accepted    | start requested
PASS | duplicate returns original     | accepted=True; cached state=2; same response=True
PASS | START RUNNING rejected         | rejected: mission not READY
PASS | live RUNNING reaches UI feed   | GCS state=3
PASS | RUNNING display within period  | rover→GCS state delay=0.3036s (2 Hz + 50ms transport margin)
PASS | real rover moves               | odom speed=0.1000m/s
PASS | UI exit does not stop rover    | UI PID=96273; rover alive, mission heartbeat, speed=0.0600
PASS | KILL active accepted           | accepted: kill requested (software stop, not a physical E-STOP)
PASS | KILL E_STOP then STOPPED       | states=[0, 1, 2, 3, 7, 8]
PASS | KILL final velocity zero       | cmd=0.0; odom=0.000000
PASS | KILL inactive rejected         | rejected: mission already inactive
PASS | timeline retains short states  | GCS transition events=[1, 2, 3, 7, 8]
PASS | live Qt map/camera render      | last render=('36', '81', '3')
PASS | live UI reception              | snapshots/camera/elapsed/Hz=('36', '81', '20.00', '1.80', '4.05')
PASS | GCS processes healthy          | []
PASS | stopped sources marked stale   | stale_sources=['candidates', 'observed_graph', 'path_status', 'perception_status', 'safety_status', 'selected_path']; odom remains fresh
PASS | owned process group cleaned    | live PIDs=[]
GCS D-11/D-12: PASS (26/26)
```

### Bag/replay regression

Command: `tools/lab_env.sh check-bag`; exact final rows:

```text
PASS | STOP while idle rejected       | not recording
PASS | recorder wrong run rejected    | run_id mismatch
PASS | record START accepted          | /ws/bags/check-stage2/73760bb2/20261119T124213_da90b349
PASS | duplicate START original reply | /ws/bags/check-stage2/73760bb2/20261119T124213_da90b349
PASS | different command ID rejected  | request_id reused with different command
PASS | second START rejected          | already recording
PASS | recording status active        | state=RECORDING
PASS | recorded KILL accepted         | accepted: kill requested (software stop, not a physical E-STOP)
PASS | recorded rover STOPPED         | state=8
PASS | record STOP closes cleanly     | stop requested; close=0.170s
PASS | duplicate STOP original reply  | stop requested
PASS | second STOP rejected           | not recording
PASS | recording status STOPPED       | STOPPED
PASS | bag metadata contract          | {"closed_cleanly": true, "code_config_sha256": "88408f30fa6c91f3add13984bfc98654c5b761880d50bec13859cd68b8ebc4bc", "environment": {"compression": "zstd_fast", "gz_partition": "bag_record_96479", "hostname": "9704990b1686", "platform": "Linux-7.0.0-42-generic-x86_64-with-glibc2.39", "ros_distro": "jazzy", "ros_domain_id": "84", "storage": "mcap"}, "params_file": "/ws/install_lab/line_follower_bringup/share/line_follower_bringup/config/competition.yaml", "recorder_exit_code": 0, "recorder_requests": [{"accepted": true, "command": 0, "reason": "/ws/bags/check-stage2/73760bb2/20261119T124213_da90b349", "request_id": "ae400876-72e3-4764-b401-59fd9377075a", "sent_stamp_ns": 1224000000}, {"accepted": false, "command": 0, "reason": "already recording", "request_id": "09adc687-f5f6-4f50-a40e-d1762b4fe7b0", "sent_stamp_ns": 1236000000}, {"accepted": true, "command": 1, "reason": "stop requested", "request_id": "9f9daaa0-30c9-4fbe-9e39-bccaa2e11403", "sent_stamp_ns": 19004000000}, {"accepted": false, "command": 1, "reason": "not recording", "request_id": "1280815f-893c-470b-8c33-c8af41495722", "sent_stamp_ns": 19176000000}], "run_id": "run_20261119_124209_897976267555384", "schema_version": 1, "start": {"ros_ns": 1224000000, "utc": "2026-11-19T12:42:13.103499+00:00", "wall_monotonic": 897980.193672033}, "stop": {"ros_ns": 19008000000, "utc": "2026-11-19T12:42:31.049550+00:00", "wall_monotonic": 897998.139727662}, "stop_reason": "stop requested", "topics": ["/camera/image_raw", "/camera/camera_info", "/clock", "/imu", "/odom", "/localization/estimate", "/perception/line", "/perception/status", "/path/observed_graph", "/path/memory", "/path/local", "/path/candidates", "/path/status", "/control/cmd_vel_candidate", "/cmd_vel", "/mission/state", "/safety/status", "/perception/obstacles", "/mission/gcs_snapshot", "/gcs/command_events", "/gcs/recording_status"], "use_sim_time": true}
PASS | expected topics and counts     | missing=[]; counts={"/camera/camera_info": 454, "/camera/image_raw": 454, "/clock": 3745, "/cmd_vel": 749, "/control/cmd_vel_candidate": 456, "/gcs/command_events": 1, "/gcs/recording_status": 75, "/imu": 2996, "/localization/estimate": 1049, "/mission/gcs_snapshot": 30, "/mission/state": 160, "/odom": 749, "/path/candidates": 454, "/path/local": 456, "/path/memory": 456, "/path/observed_graph": 456, "/path/status": 456, "/perception/line": 454, "/perception/status": 454, "/safety/status": 749}
PASS | full mission event sequence    | ['STARTUP_WAIT', 'CHECKING', 'READY', 'RUNNING', 'E_STOP', 'STOPPED']
PASS | KILL command event             | 16 total events
PASS | CLI uses shared extractor      | JSON=/ws/bags/check-stage2/73760bb2/analysis.json; exit=0; stderr=
PASS | bounded recording artifact     | bag=/ws/bags/check-stage2/73760bb2/20261119T124213_da90b349; total=10.30 MiB; duration=14.980s
PASS | UI bag RUNNING scrub | t=13.777s; map observed/planned; camera=640x480; commands disabled
PASS | UI bag end scrub | state=STOPPED; events=16; same shared extractor
PASS | UI LIVE return | replay ended; seek disabled
PASS | real bag offscreen scrub       | exit=0; stderr=QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-ubuntu'
PASS | owned process group cleaned    | live PIDs=[]
PASS | replay never publishes cmd_vel | max publishers=0; received=0
PASS | replay command redirected      | /replay/cmd_vel_blocked count=502; nonzero=373
PASS | perception/path regenerated    | {"/path/local": 492, "/path/memory": 450, "/path/status": 492, "/perception/line": 447, "/perception/status": 447}
PASS | replay excludes Gazebo bridge  | nodes=['checkpoint_a_bag_replay', 'launch_ros_96795', 'line_detector', 'mission_manager', 'motion_controller', 'path_planner', 'perception_container', 'safety_guard']
PASS | owned process group cleaned    | live PIDs=[]
D-13 RECORD/REPLAY: PASS; artifacts=/ws/bags/check-stage2/73760bb2
```

### Camera-rate regression

The simultaneous best-effort C++/CLI observers remain lossy; reliable recorders/checker receive the full stream. This is the unchanged D-05 transport policy, with the D-06 workload.

| Variant | Raw images | Topic interval s | Camera Hz | Detector status Hz | Reliable Python Hz |
|---|---:|---:|---:|---:|---:|
| standalone | 699 | 23.091 | 30.228 | 29.391 | 30.297 |
| composed | 708 | 23.529 | 30.048 | 30.219 | 30.293 |

Full log: `log_lab/d06/check-camera-rate-final.log`; raw rate JSON: `log_lab/d05-camera/results.json`.

## Limits and cleanup

Verified scope is flat, slow motion and the unchanged GCS contract. Faster 0.25 rad/s map spread failed and is not claimed fixed; 0.1 rad/s passed the original bound. No EKF/yaw fusion, hill/full-line-loss fallback, D-08 branch strategy or complete autonomous track following is claimed. Canonical association is heuristic; far-field detector fragments can reach the segment cap, and this is not global SLAM. Fusion confidence/uncertainty growth are conservative engineering baselines, not calibrated statistical guarantees. The SDK shutdown mitigation is launch-level library retention, not an upstream source-line fix.

Final process inspection (`log_lab/d06/final-processes.log`) shows only container init/sleep and the inspection command, with no Gazebo, launch, GCS, recorder, player or debugger left running. Bags remain in existing ignored/bounded check directories (one D-13 recording, two short compressed camera-rate recordings); debugger/diagnostic artifacts are ignored under log_lab. Only owned groups in line_follower_lab were signalled; no host project, verification container or unrelated container was touched. No required final check remains unverified.

## Files added/changed

- `README.md`
- `docs/test_plan.md`
- `docs/d06_evidence.md`
- `src/line_follower_bringup/launch/nodes.launch.py`
- `src/line_follower_bringup/config/competition.yaml`
- `src/line_follower_bringup/config/record_topics.json`
- `src/line_follower_navigation/CMakeLists.txt`
- `src/line_follower_navigation/package.xml`
- `src/line_follower_navigation/include/line_follower_navigation/local_path_memory.hpp`
- `src/line_follower_navigation/include/line_follower_navigation/pose_estimator.hpp`
- `src/line_follower_navigation/src/local_path_mapper.cpp`
- `src/line_follower_navigation/src/local_path_memory.cpp`
- `src/line_follower_navigation/src/pose_estimator.cpp`
- `src/line_follower_navigation/src/path_planner_node.cpp`
- `src/line_follower_navigation/test/test_mapping_memory.cpp`
- `src/line_follower_tests/test/test_component_launch.py`
- `src/line_follower_tests/test/test_pipeline_launch.py`
- `tools/d05_capture.launch.py`
- `tools/lab_env.sh`
- `tools/check_teardown.py`
- `tools/d06_capture.launch.py`
- `tools/check_d06.py`
- `tools/check_bag.py`
- `docs/d06/corner_topdown.png`
- `docs/d06/measurements.json`
- `docs/d06/rotate_topdown.png`
- `docs/d06/straight_topdown.png`
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

The D-05 PNG/JSON files were regenerated by its required regression command; provided model/world assets were not edited.

Directory-creation follow-up: output writers now create missing parents (D-06 samples/plots, D-05 plots, bag roots); teardown already uses recursive mkdir and checkpoint/GCS logs use an allocated temporary directory; `tools/lab_env.sh check-d06` passed with both `log_lab/d06` and `docs/d06` absent, created all 7 expected files, and restored the original evidence (73.52 s; `log_lab/d06-directory-audit-20261119T132148Z/result.json`, full log alongside; lead also verified two clean-copy runs).
