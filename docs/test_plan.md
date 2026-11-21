# Test plan

Source of truth: `design.md` §11 (T-01 … T-28) and §17 (D-01 … D-16).

| Test | Where | Status |
|---|---|---|
| T-02 provided assets unchanged | `line_follower_tests/test/test_asset_integrity.py` | automated |
| T-03 no driving before startup delay | `test_mission_state_machine.cpp`, `test_pipeline_launch.py` | automated (node level, no Gazebo) |
| T-04 auto start after delay + sensors | `test_mission_state_machine.cpp`, `test_pipeline_launch.py` | automated (node level, no Gazebo) |
| T-05 no world-scoped runtime interfaces | `test_static_policy.py` | automated; only dedicated dev capture harness may teleport |
| T-12 stop on input loss | `test_safety_policy.cpp`, `test_pipeline_launch.py` | automated (node level) |
| T-13 start T not treated as finish | `test_mission_state_machine.cpp` (FinishJudge) | dummy only |
| others | — | TODO |

Algorithm/performance thresholds remain placeholders until measured (design §14.2).
Checkpoint A drive/safety acceptance tolerances are specified below.

## Checkpoint A headless acceptance

Build first with `tools/lab_env.sh build`; run all checks with `tools/lab_env.sh checkpoint-a`.
This command runs `test`, `check-sensors`, `check-motion`, `check-start-stop`, and `check-guard-exit` in sequence,
reports each outcome, and exits nonzero if any fails. Individual commands are repeatable:

| Design gate / tests | Command | Assertions |
|---|---|---|
| D-01, T-01/T-03/T-04/T-12 (node level) | `tools/lab_env.sh test` | Five core nodes exchange fake sensor/state messages; graph, mission and safety unit tests; static policy and asset integrity; dev measurement predicates |
| D-02, T-01/T-02/T-05 | `tools/lab_env.sh check-sensors` | Real server-only simulation: all five sensor/time topics have their expected types and deliver messages during a 5 s sample; image 640×480 and ≥5 Hz wall time (prints measured Hz, target 30); command bridge subscriber; graph/config have no prohibited scoped bridge; 12 SHA256 assets |
| D-03, basic drive | `tools/lab_env.sh check-motion` | Sim and bridges only: forward/backward, left/right forward arcs, both in-place rotations; pose deltas in initial heading frame, yaw sign, measured twist; stop and 2 s hold after every primitive |
| D-04, T-03/T-04/T-12 (system level) | `tools/lab_env.sh check-start-stop` | Unmodified competition launch default delay: every observed pre-RUNNING command exactly zero in all six components, wall and mission elapsed ≥20 s, stationary odom; real forward motion; SIGINT only the controller PID identified in the owned launch log; bounded command/physical stop and candidate-stale reason |

Motion thresholds: directional displacement/yaw must exceed 35% of commanded speed × 1.2 s;
measured peak twist must be 35–180% of commanded magnitude. Straight motion lateral/yaw errors
are limited to 0.02 m / 0.15 rad; in-place translation to 0.02 m. Every stop must reach
linear <0.008 m/s and angular <0.02 rad/s within 1 s, then hold those speeds for 2 s with
translation <0.005 m and yaw drift <0.02 rad. Maximum excursion is reported and limited to
0.24 m during execution (completion bound 0.25 m); the nominal route is below 0.10 m.
These tolerances allow physics/transport startup without accepting a wrong direction or no motion.

The system gate permits pre-start translation <0.01 m. On controller loss, the command deadline
is the configured candidate timeout plus two control periods plus 0.06 s (currently 0.30 s);
zero must persist through the observation tail. Physical speed must settle below the motion-stop
epsilons within 1.5 s and remain there. The run ends about 2.2 s after interrupting the controller,
well before the hill. Mission/command/odom subscriptions exist before launching Gazebo; odom
pre-start samples are ordered using ROS source timestamps and commands using callback wall time.
No pre-transition command guard band is excluded.

Each run uses domain 79 and a unique Gazebo partition, records its own launch log under container
`/tmp/checkpoint-*`, and cleans only its own process group. Avoid simultaneous acceptance checks
because they share the ROS domain; domain 78 isolates colcon launch tests. Checks use wall-time
deadlines, so an overloaded machine can legitimately fail a motion/timeout requirement even if
its camera exceeds the deliberately tolerant 5 Hz floor. The world/model manifest, competition
launch contract, package dependency graph, and design §17 checkboxes are unchanged.

## Development GCS stage 1 (D-11 / D-12)

| Gate / tests | One-line command | Coverage |
|---|---|---|
| D-11 / T-26, D-12 / T-27 | `tools/lab_env.sh check-gcs` | Pure model/adapter + Qt offscreen tests, then real `dev_sim headless:=true gcs:=true auto_start:=false`: snapshot equality/rate, odom/source geometry, early/wrong-run/manual/duplicate/inactive command cases, missing recorder timeouts, snapshot-only link loss/recovery, KILL→E_STOP→STOPPED→zero, live Qt/camera telemetry |
| D-11 logic | `tools/lab_env.sh test` | C++ policy tests: READY/manual START, auto-start rejection, pending START, wrong run, duplicate action suppression and original response, bounded cache including rejections, invalid/empty requests, inactive KILL and KILL overriding pending START; C++ path transform/source/time preservation |
| D-11 node integration | `tools/lab_env.sh test` | `test_gcs_launch.py`: fake sensors + five rover nodes + bridge/broker; odom conversion, manual START, repeated original reply, UI broker KILL and short-state transition delivery; original pipeline test also verifies competition has no snapshot publisher |
| D-12 display | `tools/lab_env.sh test` | Synthetic window at 1600×1000, exact four grid spans, all four map layers, camera frame update, RUNNING state, command button routing, replay mode/seek and live-feed isolation; plain model tests for events, timestamps and short transitions; adapter tests hide stale/invalid geometry and split unknown gaps |
| Host display smoke | `tools/lab_env.sh gcs` then `tools/lab_env.sh gcs-stop` | Opens the real window on host X while Gazebo stays headless; default 20 s gate; close-window/Ctrl-C/managed-stop lifecycle |

The automated system check uses domain 82 and a unique Gazebo partition. It expects snapshot rate
1.7–2.3 Hz around the configured 2 Hz, relay latency below 0.5 s, and RUNNING state propagation
within one 0.5 s period plus 0.05 s transport/observer margin. It pauses only snapshot publishing
while the mission is still waiting for manual START, expects LOST after 2.0 s (allowed 1.9–2.4 s),
and checks that all five rover nodes and mission heartbeats remain alive. After restoring snapshots,
it starts and briefly drives the robot, then KILLs it; the inherited 0.24 m excursion guard prevents
it reaching the hill. Required snapshot/camera receive and Qt-render counts are printed from real
GCS logs. A camera limit of 6 Hz is a maximum, not an exact rate, because source-frame intervals
and wall-time throttling typically yield about 5 Hz. Every check exits nonzero on any failed row and
cleans only its owned launch process group.

`colcon test` includes the offscreen Qt test without Gazebo, so no GUI/X server is required for unit
checks. Pure model/adapter modules import neither ROS nor Qt. Qt runs in the main thread; the ROS
executor runs in a background thread and emits queued signals with copied camera images.
Mission/mode/safety transition arrays (each bounded to 32 between publications) prevent brief states
from disappearing at the 2 Hz snapshot boundary. Timeline events retain ROS stamps and run IDs.
Link/command watchdog timers use steady time even if simulation time stops.

D-13 recording/replay is now available (see below): Bag START/STOP use the new `BagCommand` contract at
`/gcs/recorder/command`, show timeout when explicitly disabled/absent, and do not crash. Replay/seek are enabled for loaded bags.
No screenshot parameters, command-line flags, or PNG deliverable are included, per the user's scope
correction. Live host capture is handled by the lead. Design checkboxes and provided assets remain
unchanged; the competition package closure still excludes `line_follower_gcs` and Qt.


## Stage 2: D-13 and independent-review follow-ups

| Gate / tests | Command | Coverage |
|---|---|---|
| D-04 exit safety | `tools/lab_env.sh check-guard-exit` | Default 20 s gate, actual motion, identity/owned-group checked safety_guard PID only receives SIGINT; last latched command is exactly zero, publisher disappears, odom settles within 1.5 s and stays still |
| Checkpoint A regression | `tools/lab_env.sh checkpoint-a` | Includes guard exit, tightened immediate controller-loss bound (candidate timeout + 2/control_rate + 0.06 s), straight lateral/yaw rejection cases, buffered PID retry and ≤1 s/100 ms group-reaping poll |
| D-11/D-12 review fixes | `tools/lab_env.sh check-gcs` | GCS UI-only exit while RUNNING leaves all five rover nodes/mission heartbeat/driving alive; stopping planner/perception/safety publishers marks all corresponding sources stale while odom remains fresh |
| D-13, T-16/T-28 analysis support | `tools/lab_env.sh check-bag` | Starts a real compressed recording by service, tests reject/de-dup cases, captures startup→RUNNING→KILL→STOPPED, cleanly stops it, checks exact active topics/counts and metadata SHA, extracts text/JSON events, scrubs real bag map/state/camera in Qt, then replays sensors through the pipeline with zero final-command publishers/messages |
| Logic / replay policy / cleanup | `tools/lab_env.sh test` | Original tests plus source freshness, wrapped headings, changed-command/expired request IDs, candidate quality/NaN and stale display, recorder cache/events/fingerprint, live/replay isolation, positive replay sensor allowlist, and PID identity/owned-group SIGKILL fallback |

The original guard-exit check really failed before the fix: the last command was nonzero and odom
remained at 0.070 m/s. Keep `log_lab/stage2-guard-exit.log` as red-state evidence. The repaired guard
uses its own minimal signal flag, ends its single-threaded executor, cancels the timer, sends five
zeros while the ROS context is valid, waits for reliable DDS acknowledgement, then shuts down.
Neither the check's zero condition nor its 1.5 s physical-stop tolerance was loosened.

The D-13 check records ≥ one real RUNNING GCS snapshot, issues KILL quickly to stay off the hill,
then captures STOPPED before closing. MCAP chunk compression keeps the sample small; the check
requires total artifacts <128 MiB. Only completed prior check directories bearing `check_complete`
are removed; active or user recordings are untouched. The recorder also has normal-operation size
and wall-time guards. Its metadata records the source/config fingerprint without requiring git.
The optional obstacle topic is absent when its extension is disabled; every other configured topic
must have messages. Recorded command topics are deliberately present in the bag so replay isolation
is tested against a bag containing real drive/KILL messages, rather than a sensor-only fixture.

The shared extractor retains every raw mission transition and emits change-only safety/mode/health
updates with run IDs and relative bag time. Perception unhealthy/fallback extraction is unit-tested;
the real OpenCV baseline remains healthy and normally has no fallback. UI scrubbing uses an actual
reader worker, checks RUNNING with observed/planned map layers and a 640×480 camera, seeks to STOPPED,
and returns LIVE. Raw mission/odom indexes refine the 2 Hz snapshot so short state transitions
are visible at the selected time; seek positions and event clicks preserve millisecond timing. A
regression test prevents cached snapshots from being mutated during seeks. It does not start a player or publish recorded commands. Smartphone video sync
is optional and is not implemented. No screenshot features or PNG deliverables are added.


## Component composition acceptance

| Check | Command | Assertions / evidence |
|---|---|---|
| Construction and options | `tools/lab_env.sh test` | Dynamic factories for detector, planner and obstacle load with NodeOptions, both intra-process defaults; standalone wrappers remain registered |
| Default launch and parameters | `tools/lab_env.sh test` | Fake-sensor pipeline has detector + planner in the default container; original named YAML selectors load nondefault calibration/lookahead; camera capture stamps match line, graph, selected path and graph points |
| Bridge selection | `tools/lab_env.sh test` | Competition/dev_sim select exactly one image source for composed vs measurement baseline, including capitalized bool arguments |
| Real Gazebo gate | `tools/lab_env.sh checkpoint-a` | Sensor check now runs the composed competition graph with only test speed capped to zero; 640×480 image rate, composed camera publisher/QoS and component CLI list; original drive/start/stop/guard-exit tests remain |
| External DDS consumers | `tools/lab_env.sh check-gcs` and `tools/lab_env.sh check-bag` | Camera relay/UI, mission/safety/status consumers and recorder still receive public topics; stale-source fault injection unloads detector/planner components through the manager API instead of looking for standalone PIDs |
| Replay | `tools/lab_env.sh check-bag` | Default container has detector + planner, no camera/Gazebo bridge; sensors from bag regenerate outputs and final velocity remains redirected with zero real-command publishers/messages |
| Before/after sample | `tools/lab_env.sh measure-components` | Same world/config/workload, 20 wall seconds while RUNNING, 0.001 m/s speed cap; detector callbacks and line Hz, p50/p95 existing latency field, per-process CPU and actual intra-process callback flags |
| Production closure | `docker build -t line-follower-components:local .` | Dockerfile's Release build with packages-up-to bringup and BUILD_TESTING=OFF succeeds; no dependency from competition onto GCS/Qt |

The selected deployment uses `ros_gz_bridge::RosGzBridge` for the image inside a four-thread container,
not a fallback image process. The image config is separate from the unchanged non-image rover bridge
config. SensorData image and default application QoS are volatile/KeepLast, compatible with rclcpp
intra-process operation. RMW endpoint discovery can report UNKNOWN history; the declared QoS and
successful intra-process construction/actual callback flags establish the local history policy.

The planner deliberately retains one mutually-exclusive group for pose/IMU/observation/timer state;
its group is independent of the detector's image group. This avoids cross-component executor
starvation and preserves existing planner synchronization. It does not promise parallel execution
inside the planner or end-to-end zero allocation. External consumers still use DDS as required.

The measurement uses status publications as one-per-image-callback receipts, rather than introducing
a heavy external camera reader. CPU is process utime+stime over 20 wall seconds (100% = one core),
excluding Gazebo, the unchanged rover bridge, controls and the observer. The composed total includes
the camera bridge and component manager; the baseline total sums the image_bridge, detector and
planner processes. ROS simulation-clock latency is quantized and is not a wall-clock transport trace.
The baseline also uses the generated standalone wrappers of the same refactored code, so algorithm
and publication logic are held constant. Results include the bridge implementation change and are
not attributed solely to serialization. No performance improvement is assumed.

Component loading is asynchronous: tests wait for the exact populated node list, not merely the
manager service's existence. CLI inspections use `--no-daemon --spin-time 3` for discovery and leave
no daemon. Original provided worlds/models/manifest and design checkboxes are untouched.

## D-05 / T-06, T-07, T-08: image-ground-truth detection

`tools/lab_env.sh check-d05` starts a fresh headless stationary sim + the default three perception
components, on domain 92 with a unique Gazebo partition. The dev-only `tools/check_d05.py` is the
single narrowly named exception to the scoped-service static scan; a separate policy test prevents
rover sources from importing it or the mesh oracle. Every other runtime/config/launch file remains
subject to T-05. No mission/control nodes or final-velocity publishers run in this capture graph.

`tools/d05_groundtruth.py` reads consecutive strip-triangle pairs from the provided STL, verifies
2 cm strip edges, extracts their midpoint centerlines, and applies world/model/link/visual poses.
It never imports detector code. Seven mesh-derived named poses cover both T marks, a straight,
a right-angle approach, a circular curve, the crossing, and hill entry. After an owned simulator
teleport and 2 s settle, each 4 s sample reports input/output Hz and p50/p95 processing ms. Images
and observations are joined by exact capture stamp. Results/PNG overlays/top-down plots go to
`docs/d05/`; no source asset changes.

Flat gates: median/p95/max centerline distance <0.025/0.045/0.080 m; ≥80% of flat mesh centerline
samples in camera FOV and the observation's reported radial range lie within 0.030 m of detected
polylines. Corner/crossing/T landmarks must have 2/4/3 incident segments within 0.050 m of the mesh
landmark. Projection must be valid. These thresholds were fixed before the first run; subsequent
recall correction uses the specified **reported range**, rather than including unrelated lines
beyond that range. Hill results never count as a flat-ground success and do not gate D-05/D-07.
The forward projection cap is 1.2 m; reported ranges are radial, so lateral points can exceed 1.2 m.

C++ synthetic tests cover empty/white images, a straight, both sharp right angles, an arc, an X,
a T, an image-edge line, faint distant transverse strokes, known-pixel projection, small capture
pitch and slope invalidation. Navigation tests require untrusted projection to stop the planner
and stay out of observed memory. Fake launch-test images now contain a real 640×480 black line;
white dummy-only stimuli are no longer sufficient to make the rover drive.

`tools/lab_env.sh check-camera-rate` compares standalone/composed image transports with simultaneous
20 s C++ SensorData, rosbag, Python CLI and reliable checker observers (domains 90/91). `PROBE_RELIABLE=1` inside
the container selects a reliable C++ reader; `RATE_COMPOSED_ONLY=1` limits the measurement to the
composed variant. SDK `SERVICES` selects reliable/volatile/KeepLast camera QoS, depth 5. Recordings
use MCAP zstd and fixed owned `bags/camera-rate-*` directories, replacing only previous camera-rate
artifacts. `ros2 topic hz` in this Jazzy image has no QoS-option flag; its observed rate is not a
publisher-rate proof. Raw evidence is in ignored `log_lab/d05-camera/`.

Run `build`, `test`, `checkpoint-a`, `check-gcs`, `check-bag` and the competition Docker build after
D-05. Safety thresholds and the default 20 s gate are unchanged. Full Fast-SCNN work is D-15.

## D-06 / T-09 / T-10: capture-time odom graph and moving local memory

`tools/lab_env.sh check-d06` runs three independent server-only simulations on domain 95 with unique
Gazebo partitions. It loads bridges, the three perception components, and mission_manager only for
its GCS snapshot (auto-start false). No controller or safety_guard runs. The harness is the sole
`/cmd_vel` publisher, repeatedly publishes zero on all exits, and cleans only its process group.
Placements derive from the immutable mesh oracle and reuse the D-05 dev teleport helper; no new
scoped simulator interface is added to rover code. It records the initial odom-to-world rigid transform
and resets local memory after placement to exclude pre-teleport data.

Scenarios: forward ~0.7 m at 0.07 m/s; ~0.39 m approach to a right-angle corner at 0.06 m/s until its
vertex is ~0.21 m ahead (inside the measured ~0.316 m blind zone); in-place +0.5, -0.5, then 0 rad
headings at 0.1 rad/s. Thresholds: observed median/p95/max <0.030/0.055/0.100 m, memory p95/max
<0.055/0.100 m, same-line normal spread <0.015 m, storage ≤1,200 points / 32 segments and end sample
<600 points, no UNKNOWN, frame `odom` for both graphs and base frame for local path. Localization
must match timestamp-matched reliable odom XY within 0.002 m and native Gazebo true heading within 0.005 rad (D-07 gyro mode); the 2 Hz snapshot must match localization within 0.015 m / 0.03 rad. Stops retain zero final command and <0.008 m/s / <0.02 rad/s odometry speeds.

The corner must retain a MEMORY_UPDATED vertex within 0.015 m and an outgoing observed-before leg.
Its leg angle stays 75–105 degrees; tangents are measured at least 0.06 m from the vertex to avoid
quantization and duplicate shared-vertex samples dominating a two-point angle. Straight local memory
must cover near points below 0.12 m and beyond 0.18 m inside the blind zone after moving. Normal spread
uses direction-compatible segments of the same straight, excluding legitimate transverse crossing
legs. PNGs show mesh GT, direct/retained geometry, memory in the local path, trajectory and headings:
`docs/d06/{straight,corner,rotate}_topdown.png`.

C++ tests cover interpolation including yaw wrapping and outside/old rejection; both-sensor health;
known odom transform and UNKNOWN rejection; repeated-view deduplication; age/travel uncertainty,
expiry/passed pruning; run/fault reset; capacity; current-base blind-zone and sharp corner geometry;
a moving observation window retaining fixed old points; shared corner vertices under repeated noisy
views. Node-only tests check capture stamps per point and the separate odom basis stamp on local paths.
The public reset hook enables deterministic fault/reset and dev teleport tests.

`tools/lab_env.sh check-teardown` requires 10 camera-running composed competition launches to report
normal component-container exits and empty owned groups after SIGINT. It caught the old -11 exit.
The bridge DSO preload is conditional on loading the SDK camera bridge; replay retains no Gazebo
component/preload. The full regression set is build, test, check-teardown, check-d06, check-d05,
checkpoint-a, check-gcs, check-bag, check-camera-rate, and the production Docker build. Replay now
also checks regenerated `/path/memory` while still requiring zero `/cmd_vel` publishers/messages.

The 0.25 rad/s investigation failed spread with wheel-odom vs IMU-yaw disagreement about 0.045 rad.
At 0.1 rad/s it fell below 0.01 rad and the original spread gate passed. No IMU yaw fusion or EKF was
added, and no safety tolerance was loosened. Faster rotation accuracy, slopes/full-loss fallback,
branch decisions and complete autonomous tracking remain later stages. Exact results and earlier
failures: `docs/d06_evidence.md`; §17 checkboxes remain for the lead.

## D-07 / T-07, T-09, T-10, T-23, T-24, T-25

`tools/lab_env.sh check-d07` uses domain 97 and a unique Gazebo partition per scenario, server-only, without motion_controller or safety_guard. Its dev publisher sends zero on all exits and owns only its launch process group. Native Gazebo truth stays inside the offline harness; no world topic is bridged.

The command compares odom-only and gyro heading against model pose at ±0.25/0.5 rad/s, gates the chosen source at map spread <0.015 m and true-heading p95 <0.01 rad, traverses the entire hill at 0.1 m/s and a flat finish-T line end at 0.06 m/s. Confirmed observed errors must satisfy median/p95/max <0.025/0.045/0.080 m; memory p95/max <0.055/0.100 m. It requires pitch-invalid slope observations, memory/slowdown/stop/reacquisition and sustained recovered vision after the exit, no UNKNOWN or too-uncertain/expired confirmed points, and slowdown/stop with a positive verified reserve at the blind end. The harness deliberately continues after the planner requests STOPPING; autonomous following is not being claimed.

Gtests cover policy transitions, monotonic speed caps, mismatch/expiry/pitch/health gates, position/heading/junction comparison (including newly visible connections), gyro integration/reset, long-edge rejection, and connected parallel-path retracing. The D-06 regression now verifies localization XY against odom, heading against Gazebo true pose, and GCS pose/path conversion against localization; existing tolerances remain unchanged. Final acceptance includes D-05/D-06, Checkpoint A, GCS/bag/replay/camera-rate/10-run teardown and production Docker build, plus D-07 and D-06 runs with their output directories moved aside. See `docs/d07_evidence.md` for exact commands and results.

## D-08 / D-09: autonomous section and full-track gate

`tools/lab_env.sh check-drive` runs hill, right-angle corner, circular curve and crossing sections,
then two default `competition.launch.py headless:=true` runs with the unchanged 20 s startup gate.
The harness uses domain 98 and a unique Gazebo partition per run. Sections shorten the startup gate
and teleport through a dev-only launch; all movement comes from the real planner/controller/guard.
The observer publishes no velocity command. KILL stops each run and owned process groups are cleaned.

The offline mesh walker follows endpoint connectivity, retaining straight order at the self-crossing.
Progress is ordered XY arc length; the current nearest-edge search is restricted to the neighbourhood
of prior progress. A lateral error above 0.08 m for 0.6 s fails; no progress for 25 s or the 600 s run limit also fails. Sections require traversing their full prescribed span;
full completion means entering the finish area within 0.12 m arc distance, not D-10 finish detection.
Any failed run exits non-zero. JSON includes selected reasons, modes, errors and the diagnostic bag.
PNG plots and timelines are written to `docs/d0809/`; every output parent is created automatically.

Unit tests cover bottom-image continuity/ambiguity and steering signs, relative hill elevation trust,
shadow rejection, straight-through branch locking, local curve tangents, sharp corner target/rotation,
and preservation of valid metric memory before image fallback.
Selected-route tests verify retained ordered corners, strict source/expiry/uncertainty, no self-renewal, no jump to a nearby return leg, bounded storage and replacement on a fresh contradictory selection. The launch tests verify missing/stale feedback, measured stop-before-rotate, bounded angular output, timeout zero, same-target retry rejection, one genuinely changed-target retry, forward-progress episode reset, and new-run reset. Existing D-05/D-06/D-07 ground-truth, expiry/source and safety checks remain regression gates.
See `docs/d0809_evidence.md` for the exact final results, including incomplete full-track runs.

This gate covers T-06 (shape following), T-07 (hill fallback), T-08 (straight-through crossing) and
T-11 (stop/rotate transitions). Observed-leg selection and new-straight-leg correction of an early turn lock are unit-tested. `--section crossing_east` additionally covers the first traversal of the crossing and the following blind bend; `--section left_bend` covers the later left-side curve. The original crossing section covers the later northbound traversal. Full-run bags record partial-view decisions and their correction; see the evidence for observed outcomes.

D-10 / T-13: `tools/lab_env.sh check-d10` runs the unchanged competition launch (20 s gate) for start-T rejection and two full-track observations, plus a manual-start section beginning 1.35 m before the finish bar. The section retains the default minimum mission travel and start-departure requirements. Ground truth is derived from the immutable mesh in the dev observer only. FINISHED must hold drive disabled, the last commands must be zero, odom speed must be below 0.01 m/s, and the robot center must stop within 0.08 m of the bar with lateral error below 0.08 m. Existing off-line gating remains 0.08 m for 0.6 s. `--only start`, `--only finish`, or `--only full --full-runs 2` isolate scenarios; artifacts are under `docs/d10/` and `log_lab/d10/`. Unit tests cover capture interpolation, start/return-to-start rejection, duplicate and inconsistent evidence, one-frame rejection, blind-zone crossing, stale evidence, side passes, and FINISHED holding.
