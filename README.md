# ROS 2 Line Follower Rover

TurtleBot3 Waffle Pi line follower for ROS 2 Jazzy + Gazebo Harmonic. Design: [`docs/design.md`](docs/design.md) (v1.2).

**Status: real C++ OpenCV line detection, composed image pipeline, Checkpoint A and development GCS.**
The detector segments black strokes, extracts a centerline graph and projects capture-time observations
with camera geometry and IMU attitude. Capture-time odom mapping and bounded local memory are implemented; branch decisions and mission completion still include staged placeholders;
full autonomous track completion is not yet verified.

## Run

```bash
colcon build
source install/setup.bash
ros2 launch line_follower_bringup competition.launch.py            # official one-liner
ros2 launch line_follower_bringup competition.launch.py headless:=true
ros2 launch line_follower_bringup dev_sim.launch.py record:=true gcs:=true
ros2 launch line_follower_bringup replay.launch.py bag:=<rosbag dir>
```

## Test

```bash
colcon test && colcon test-result --verbose
tools/sim_smoke_test.sh 40        # headless Gazebo: 20 s gate, auto start, sole /cmd_vel publisher
```

## Layout

| Path | Contents |
|---|---|
| `src/line_follower_interfaces` | messages / service (design §5.2) |
| `src/line_follower_perception` | `line_detector` component + standalone wrapper — real OpenCV baseline + Fast-SCNN stub/fallback |
| `src/line_follower_navigation` | `path_planner` component + standalone wrapper — LocalPathMapper, LocalPathMemory, PoseEstimator, CandidateFeatureBuilder, rule/MLP evaluators, TrajectoryPlanner |
| `src/line_follower_control` | `motion_controller` (Pure Pursuit + rotate PID), `safety_guard` (sole `/cmd_vel` publisher) |
| `src/line_follower_mission` | `mission_manager` — 20 s gate, readiness, state machine, finish judge (dummy) |
| `src/line_follower_bringup` | `competition` / `dev_sim` / `replay` launch, parameters, bridge config; provided `worlds/` + `models/` (unchanged, guarded by `SHA256SUMS`) |
| `src/line_follower_tests` | launch-level pipeline test, static policy scan, asset integrity |
| `tools/gcs` | dev-only live/replay Qt GCS, command broker and recorder |
| `tools/bag_analyzer` | dev-only rosbag event extraction |
| `tools/sim_smoke_test.sh` | headless Gazebo smoke test of the competition launch |

## Topic graph

```
/camera/image_raw ─► line_detector ─► /perception/line ─► path_planner ─► /path/local ─► motion_controller
                                   └► /perception/status ──────────────┐            │
/imu ─► line_detector (capture-time ground projection)
/odom, /imu ─► path_planner, mission_manager, safety_guard             ▼            ▼
mission_manager ─► /mission/state ─────────────────────────────► safety_guard ◄─ /control/cmd_vel_candidate
path_planner ─► /path/status ─────────────────────────────────────►    │
                                                                       ▼
                                                                    /cmd_vel ─► Gazebo
```

## Checkpoint A in the lab container

ROS 2 Jazzy and Gazebo Harmonic run inside `line_follower_lab`; the host ROS environment
is not used. From this workspace (quote its path because the parent name ends in a space):

```bash
tools/lab_env.sh build
tools/lab_env.sh test                 # D-01: five-node graph and unit/static tests
tools/lab_env.sh check-sensors        # D-02: sensors, bridge, asset hashes
tools/lab_env.sh check-motion         # D-03: dev-only primitive drive and stop checks
tools/lab_env.sh check-start-stop     # D-04: default 20 s gate and controller-loss stop
tools/lab_env.sh check-guard-exit     # D-04: SIGINT safety_guard while moving; latched zero + stop
tools/lab_env.sh checkpoint-a         # all tests + the four checks; final PASS/FAIL
```

Each Gazebo check starts a fresh server-only launch, prints measured PASS/FAIL rows, and exits
nonzero on failure. They use ROS domain 79 and a unique Gazebo partition; unit/launch tests use
domain 78. Run these commands sequentially. Cleanup signals only the launch process group owned
by that check; the motion checker sends repeated zero commands before cleanup, including on
failure or Ctrl-C. The printed launch-log path is inside the container.

The dev-only checker (`tools/checkpoint_a.py`) is not a competition dependency. Motion tests
launch only `sim.launch.py` and publish test commands directly; the competition graph retains
`safety_guard` as its sole final-command publisher. Forward/backward, moving turns, and both
in-place rotations use 1.2 s commands at 0.04–0.06 m/s and 0.5–0.6 rad/s, with a stop check after
each primitive. An odometry excursion guard aborts at 0.24 m from the initial pose. Autonomous
backward driving and track-following algorithms remain unimplemented.

## Development GCS (D-11 / D-12)

```bash
tools/lab_env.sh build
tools/lab_env.sh gcs                       # visible 1600×1000 Qt window; Gazebo server only
tools/lab_env.sh gcs auto_start:=false     # manual START after the unchanged 20 s sensor gate
tools/lab_env.sh gcs-stop                  # from another terminal; stop this managed session
# tools/lab_env.sh stop is an alias for gcs-stop; Ctrl-C or closing the window also stops it.
tools/lab_env.sh check-gcs                 # isolated headless communication + offscreen UI checks
tools/lab_env.sh test                      # includes pure model, Qt, C++ and node-only GCS tests
```

The lab container uses the host X display for `gcs`. The window has the specified 4×4 layout:
3×3 odom map, right-side command panel and camera, and the bottom LIVE event timeline. Wheel zoom,
follow-robot and an optional display-only trail are available. Colours distinguish observed (green),
memory (cyan), selected/planned (blue), and feasible candidate segments (yellow dashed). Stale/invalid
geometry is hidden; a bundled Noto Sans CJK UI font subset renders the Korean command labels.
The map displays the real detector graph and the current planner output. The offscreen test also
exercises synthetic memory and branch layers.

`mission_manager` optionally aggregates `/mission/gcs_snapshot` at 2 Hz; `gcs_bridge` relays it to
`/gcs/snapshot`, publishes `/gcs/link` (STALE after 1 s, LOST after 2 s), and throttles the RGB camera
to at most 6 Hz on `/gcs/camera`. Qt receives queued signals from a background ROS executor. Snapshot
transitions preserve short mission/mode/safety changes between 2 Hz updates; the timeline also shows
command results. Rendering/receiving counts and rates appear as `UI_RENDER` / `UI_RECEIVE` in logs.

UI commands travel via `/gcs/command_requests` to `gcs_command`; accepted/rejected/timeout results
appear on `/gcs/command_events`. Mission commands use `/mission/command`: a bounded 256-response cache
returns the original response for matching duplicates without repeating their action. Reusing an ID
for a different command or an expired duplicate is rejected; seen IDs are retained for the run. Non-empty wrong
run IDs are rejected. START requires READY and `auto_start=false`; KILL requests E_STOP→STOPPED and
is rejected for an inactive mission. KILL requires a working connection and is a software stop,
**not a physical E-STOP**. The existing one-shot CLI remains available inside a sourced lab shell:

```bash
ros2 run line_follower_gcs gcs_command --ros-args -p command:=kill
```

The Bag buttons send `BagCommand` requests to `/gcs/recorder/command` with request/run IDs
and a send stamp. The development recorder now serves them, records compressed MCAP bags, and publishes status.
If the recorder is disabled (`recorder:=false`), absent-service requests still time out after 2 s.
OPEN BAG / REPLAY loads a bag into the timeline; the slider and event markers seek offline
map/camera/state data with millisecond seek positions. LIVE returns to the rover feed. Rover command buttons are disabled in replay.
There is no screenshot/capture feature, per the updated scope.

Competition keeps `gcs_snapshot_enabled: false`, creates no extra snapshot subscriptions/publisher,
and never loads GCS or Qt. `dev_sim gcs:=true` enables snapshots. In an already enabled development
session, setting `gcs_snapshot_enabled=false` suppresses only snapshot publishing (useful for a link
loss test); restoring true resumes it. Enabling snapshots from a competition run requires relaunching
with the launch argument, because its extra subscriptions are intentionally absent there.
Managed visible sessions use ROS domain 81, system GCS checks 82, and node-only GCS tests 83.
`gcs-stop` validates its recorded launch PID identity and signals only that launch, with an owned
process-group fallback for cleanup; it never pattern-kills unrelated sessions.


## ROS Bag recording, event analysis and replay (D-13)

```bash
tools/lab_env.sh gcs                       # Bag START/STOP and OPEN BAG / REPLAY are available
tools/lab_env.sh check-bag                 # real record → KILL → clean close → analysis → UI scrub → safe pipeline replay
# Inside a sourced lab shell:
python3 tools/bag_analyzer/run_analyzer.py /ws/bags/<recording>
python3 tools/bag_analyzer/run_analyzer.py /ws/bags/<recording> --json --output /ws/bags/events.json
ros2 launch line_follower_bringup replay.launch.py bag:=/ws/bags/<recording> rate:=2.0
```

`gcs_recorder` owns its rosbag child process group and serves `/gcs/recorder/command`. It rejects
invalid/wrong-run commands and duplicate IDs reused for a different action; exact repeats return the
original reply. `/gcs/recording_status` reports state, run, path, reason and size. MCAP uses Zstd chunk
compression, with a 300 s / 1 GiB recording guard by default. Bags go under ignored `bags/`.
`record:=true` starts the same recorder automatically; `recorder:=false` disables it for diagnostics.
The shared topic list is `src/line_follower_bringup/config/record_topics.json` (also exposed as
`dev_sim.launch.py`'s `RECORD_TOPICS`); it includes the §10.3 topics, GCS snapshots/command events,
camera info, clock and recording status. The obstacle topic is recorded only when its extension
publisher is enabled.

Each bag contains `run_metadata.json` with run ID, simulation-time setting, UTC/ROS/steady start and
stop times, source/config SHA256, environment, request audit and whether close completed cleanly.
The event extractor and UI share the same reader: events use bag-relative seconds and include
mission transitions, safety reasons, drive modes, perception health/fallback and command results.
The UI reader runs in a worker thread, refines state/pose from raw mission/odom samples, and reads
only the preceding second of camera data on a seek;
live data cannot overwrite the offline view, and offline display never publishes recorded commands.

Pipeline replay remains separate from UI scrubbing. It plays only camera/IMU/odom, regenerates the
pipeline, and redirects the safety guard's final output to `/replay/cmd_vel_blocked`. It launches no
Gazebo bridge and never plays recorded velocity or GCS command topics. `check-bag` verifies zero
`/cmd_vel` publishers/messages throughout replay, regenerated perception/path messages, and blocked
commands. Checks use domains 84 (record) and 85 (replay), retain one completed check artifact under
`bags/check-stage2/`, and print its path, message counts, metadata and event list.

The standard `dev_sim` launch lets the rover continue if the GCS window closes or crashes.
`stop_on_gcs_exit:=true` opts into stopping the whole launch; `tools/lab_env.sh gcs` uses that option
for its managed visible session. `check-gcs` exercises the default independent lifecycle while
RUNNING. Snapshots include wall-time `stale_sources` flags for pose, graph, candidates, selected path,
path/safety/perception status; the UI hides stale geometry and marks status, distinguishes LOST
(red) from STALE (amber), and displays branch rule/MLP quality with NaN-safe labels.

The safety guard now stops its executor before publishing five final zero commands and waiting for
DDS acknowledgements on SIGINT/SIGTERM. The original shutdown callback could race with its timer and
leave a nonzero command latched; the new guard-exit check caught that defect and verifies the fix
without relaxing its thresholds. Controller-loss command stop is bounded by candidate timeout +
two control periods + 0.06 s (0.30 s at the current defaults).


## Composed image pipeline (default)

The official command remains `ros2 launch line_follower_bringup competition.launch.py`.
Its default image path is one process:

```text
perception_container  (component_container_mt, 4 executor threads)
  camera_bridge  [ros_gz_bridge::RosGzBridge, image-only config]
        └─ /camera/image_raw ─► line_detector ── /perception/line ─► path_planner
```

All three components enable rclcpp intra-process communication. `line_detector` and the optional
`obstacle_detector` are registered components from `line_follower_perception`; `path_planner` is
registered by `line_follower_navigation`. Generated standalone executables keep the existing names
for `ros2 run`. The logical node names, topic names, YAML keys and message schemas remain unchanged.
The default container does not load the obstacle extension; `dev_sim obstacles:=true` adds it.

The parameter bridge for clock/IMU/odom/camera info/final velocity remains a separate process with
its original rover-only configuration. `config/camera_bridge.yaml` has exactly one image bridge,
using reliable, volatile KeepLast(5) QoS (the SDK `SERVICES` preset). Reliable external readers
repair image-fragment losses; best-effort SensorData readers remain compatible but can drop frames. The standalone `ros_gz_image` process is absent from the
competition graph. A bridges-only `sim.launch.py` still uses that external bridge for motion checks.
Replay uses a container with just detector + planner; bag play supplies the image, so replay loads
no Gazebo bridge. `dev_sim` uses the same composed default as competition.

Each component has its own default mutually-exclusive callback group. Four executor threads let
image work run alongside the planner; planner callbacks remain serialized together to protect
pose/memory state without introducing new data races. `LineObservation` and the other app outputs
publish owned messages with `std::unique_ptr`. The packaged Gazebo bridge controls image allocation;
we do not claim end-to-end zero-copy. Gazebo conversion, RGB/BGR conversion and external DDS readers
still incur their normal work. GCS and the recorder continue to receive all public topics over DDS.
Camera capture stamps remain the source for line/observed-graph headers and every point’s `capture_stamp`.
The local-path header identifies its current odometry basis; its points retain their individual capture stamps. The detector
drains active callbacks on component unload and blocks queued callbacks from accessing destroyed
state; a stress test unloads the detector and planner 80 times each while inputs arrive.

```bash
tools/lab_env.sh build
tools/lab_env.sh test
tools/lab_env.sh checkpoint-a
tools/lab_env.sh check-gcs
tools/lab_env.sh check-bag
tools/lab_env.sh measure-components      # 20 s RUNNING per variant; raw JSON + process CPU
# In a sourced lab shell while competition is running:
ros2 component list --no-daemon --spin-time 3
```

`compose:=false` is available only as a before/after measurement opt-out; the measurement tool uses
it automatically. Both variants cap test speed at 0.001 m/s to stay within about 0.021 m of spawn
during the long RUNNING sample, leaving normal parameters untouched. It estimates the detector's
camera input rate from its per-frame status stream, observes line output, reports the existing
capture-to-publish metric, and samples CPU ticks for the relevant process(es). It deliberately adds
no external raw-image subscriber. Output lives in ignored `log_lab/components/`.
The measured result is documented in `docs/components_evidence.md`; composition is not presented as
a performance improvement when the measurements do not show one.

The final sample sustained about 30.3 detector callbacks/s. CPU rose from 19.80% to 22.40% of one core,
and the simulation-clock latency p50/p95 changed from 4/4 to 4/8 ms; no performance gain was demonstrated.

## Real line detector (D-05)

```bash
tools/lab_env.sh build                  # Release build for the CPU detector
tools/lab_env.sh check-d05              # headless stationary captures + immutable STL ground truth
tools/lab_env.sh check-camera-rate      # 20 s C++ / rosbag / Python observer comparison
```

The baseline uses capped Otsu thresholding plus local contrast for distant one-pixel lines, a 4 mm
ground grid, topology-preserving thinning, explicit junction legs and sharp corner splitting.
Camera calibration, forward projection limit and slope threshold are parameters under `line_detector`.
Capture stamps propagate unchanged. Capture-time IMU attitude corrects small flat-ground pitch;
missing attitude, pitch/roll ≥2 degrees, or inconsistent projected stroke width marks projection
untrusted. Diagnostic points remain in the observation at lower confidence; the planner stops and
excludes them from observed path memory. Full hill perception/memory handling remains D-07.
Fast-SCNN remains the D-15 stub with OpenCV fallback.

`check-d05` runs only Gazebo, bridges, detector and planner: no mission/controller/velocity publisher.
Its dedicated dev harness may teleport its own stationary robot; the production static policy still
prohibits scoped Gazebo interfaces everywhere else. Mesh-derived geometry and test poses stay under
`tools/` and tests. It prints per-pose distance errors, recall within the reported observation range,
landmark topology, processing latency and input rate. Flat poses gate the result; hill entry is
informational. Overlays/top-down plots are in `docs/d05/`; measurements and limitations are recorded
in [`docs/d05_evidence.md`](docs/d05_evidence.md). No provided asset is modified.

## Local odom mapping and memory (D-06)

```bash
tools/lab_env.sh build
tools/lab_env.sh check-teardown          # 10 composed competition SIGINT exits, all must be clean
tools/lab_env.sh check-d06               # moving straight, blind corner, ±0.5 rad rotation
```

The planner buffers observations briefly until odometry brackets their camera capture stamp. It
interpolates position and shortest-angle yaw, rejects stale/out-of-buffer captures, and stores only
direct OBSERVED segments in `odom`. `/path/observed_graph` keeps capture-time geometry. Stable memory
IDs associate overlapping, direction-compatible observations and preserve detector-reported connections;
`/path/memory` exposes the bounded retained graph in `odom`. This topic is also recorded in dev bags.

Memory expires after 10 s, behind the rover, outside its local region, or above the uncertainty limit.
Time, travelled distance and rotation grow uncertainty. Defaults cap storage at 1,200 points / 32
segments. New run IDs, terminal/fault recovery transitions, and `/path/reset_memory` (`std_srvs/Trigger`)
clear it. The dev moving check uses that reset after teleporting; competition never uses simulator
pose services. A reset discards pending pre-reset captures and clears the published graphs/path.

`/path/local` uses the current base frame. Its header stamp is the **odometry basis stamp**; every
point keeps its **original camera capture stamp**. Retained coordinates are MEMORY_UPDATED. A matching
current raw observation supplies exact OBSERVED coordinates. Only unambiguous recorded endpoint
connections extend the local chain; no new geometry or branch choice is inferred. Shared junction
anchors retain corner structure. Mixed memory paths have a conservative 0.06 m/s cap and point/path
expiry. D-07 now carries valid memory through image/projection loss and stops on exhausted/uncertain memory or stale odom/IMU.

The existing GCS snapshot preserves these source labels for cyan memory / green observed / blue planned
drawing, with robot pose in `odom`. No UI layout changes or GCS dependency were added to competition.
`/localization/estimate` reports odom translation plus gyro heading and IMU pitch, and marks either stale sensor unhealthy using source and steady-clock receipt ages. Gyro heading is optional (`imu_yaw_enabled`); there is no EKF or fusion of correlated absolute headings. The D-06 moving check uses 0.07 m/s straight, 0.06 m/s corner approach and 0.1 rad/s rotation. The historical faster-turn heading discrepancy documented in [`docs/d06_evidence.md`](docs/d06_evidence.md) is measured and addressed by D-07 below.

The prior composed SIGINT crash is mitigated by keeping the SDK `libros_gz_bridge.so` mapped through
process shutdown using a container-only preload when the camera bridge component is loaded. It changes
no topics/QoS and is absent from replay. Existing preload entries are preserved. Camera-only/app-only
isolation, backtrace and repeated clean exits are documented in the D-06 report.

## Blind-zone and hill validity (D-07)

```bash
tools/lab_env.sh check-d07                # headless truth-based heading, hill and blind-end checks
tools/lab_env.sh check-d07 --heading-only # odom versus gyro at 0.25 and 0.5 rad/s
```

The planner uses wheel-odom translation and optional gyro-integrated heading (`imu_yaw_enabled`, enabled after the Gazebo comparison). Capture-stamped observations enter odom memory only when the ground projection is trusted. Pitch, projected stroke width and stable recovery frames guard the flat-plane assumption, including a level hilltop looking down at lower ground. Re-observation must agree in position, heading and known junction geometry before it updates memory; an edge absent from an earlier partial view remains unknown. New connections need endpoints within 4 cm, and stored junction positions must agree within 6 cm. Loss uses MEMORY_TRACKING, then SLOWDOWN/STOPPING as uncertainty grows or verified distance runs out; reappearance uses REACQUIRING before VISION_TRACKING. The speed envelope reserves 5.5 cm and assumes 0.15 m/s² braking. This implements path validity; autonomous hill/corner following remains D-09.

GCS snapshots now use the localization pose for the robot and selected-path conversion, keeping the display aligned with the planner's heading. The Gazebo checks create all output directories and save tables/plots under `docs/d07/`; details and limitations are in [D-07 evidence](docs/d07_evidence.md).

## Autonomous decisions and driving (D-08 / D-09)

```bash
tools/lab_env.sh check-drive                         # four sections, then two real competition runs
tools/lab_env.sh check-drive --full-runs 0 --limit 120 # section checks only
tools/lab_env.sh check-drive --section crossing_east  # first crossing plus its following blind bend
tools/lab_env.sh check-drive --section left_bend      # focused later left-curve check
tools/lab_env.sh check-drive --only-full --full-runs 2 # default 20 s competition gate, up to 600 s each
```

The rule-based planner follows connected observed polylines, prefers incoming-heading continuity
at a crossing, and locks the selected leg until it is passed, becomes infeasible, or a newly observed straight continuation contradicts a provisional turn. Candidate scores
and reasons remain available to GCS. Pure Pursuit follows the selected point order; sharp observed
corners request a slower approach and a stop before bounded rotation to the observed outgoing heading.
Rotation requires fresh localization and odometry and an actual stop; its eight-second budget faults
to zero for that episode. Alignment or measured forward progress ends the episode; a fresh, substantially changed observed heading permits one bounded retry after cooldown. `safety_guard` remains the sole `/cmd_vel` publisher.

When projection is untrusted, a single continuous lower-image stroke can drive bounded image-space
following (`IMAGE_TRACKING=7`, at most 0.06 m/s, 2.2 m or 50 s). This mode publishes pixel offset and
heading from a look-ahead point on the visible skeleton, without metric path points. Curves and visible corners are accepted at a lower speed; disjoint/branched strokes stop. Metric reacquisition checks remain.
An odometry/IMU relative-height check prevents the elevated hilltop from being treated as the original
flat plane. Metric memory lasts at most 30 s and retains its 0.06 m uncertainty cutoff. A separate bounded buffer (256 points / 1.5 m) preserves the order of a previously selected connected route when graph tracks fragment. It uses the same capture expiry and uncertainty gates; compatible short updates cannot erase an observed corner, and a newly selected contradictory route replaces it. It never joins disconnected strokes.

The observer alone uses the read-only mesh and simulator truth, with 0.08 m lateral error sustained
for 0.6 s defining an off-line failure. Bags are under ignored `bags/drive/`; top-down plots, timelines,
final camera frames and JSON are under `docs/d0809/`, created automatically. The current full-track
limitations and measured results are in [D-08/D-09 evidence](docs/d0809_evidence.md); section success
does not establish full-track completion. Finish detection and automatic mission completion remain D-10.
