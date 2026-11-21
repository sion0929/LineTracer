# Component pipeline implementation and evidence

Verification: 2026-11-19 (Asia/Seoul; raw container logs use UTC).
Workspace: `/home/sion/Line_Tracer /line_follower_ws`.

**Final acceptance: PASS.** As directed in coordinator message `msg_34ea311deec3`, final runs determine
PASS/FAIL; earlier development failures are listed separately below. Every final build, test and
Gazebo acceptance command exited 0. The full requested three-component variant is shipped; no image
bridge fallback was needed. **The measurements did not demonstrate a performance improvement.**

## Shipped graph and compatibility

```text
perception_container (component_container_mt, thread_num=4)
  camera_bridge  [ros_gz_bridge::RosGzBridge; /camera/image_raw only]
      └─ Image ─► line_detector [line_follower_perception::LineDetectorNode]
                    └─ LineObservation ─► path_planner [line_follower_navigation::PathPlannerNode]
```

All three descriptions pass `use_intra_process_comms=True`. The separate parameter_bridge keeps the
unchanged clock/IMU/odom/camera-info/velocity bridge configuration. Competition has no standalone
`ros_gz_image` process. Camera bridge, detector and planner share one OS process; mission_manager,
motion_controller and safety_guard remain separate processes. Topic names, logical core node names,
message schemas and normal competition parameters are unchanged. The official command remains:

```bash
ros2 launch line_follower_bringup competition.launch.py
```

`nodes.launch.py`, competition and dev_sim default to composition. Replay uses only the two app
components and explicitly disables the camera bridge and obstacle extension. Competition explicitly
disables the extension. `obstacle_detector` was also converted cheaply to a component; dev_sim can
add it to the same container with `obstacles:=true`. All three app nodes retain their original
standalone executable names using `rclcpp_components_register_node(... EXECUTABLE ...)`.
A bridges-only sim launch retains the external image bridge for the dev motion check.

Each component has its own default mutually-exclusive callback group. Four executor workers allow
heavy detector callbacks to run independently of the planner group. Planner observation, odom, IMU
and watchdog callbacks remain serialized together, preserving existing pose/memory synchronization.
This does not introduce parallel mutation within the planner or claim hard real-time scheduling.

The image configuration declares SensorData QoS (volatile, KeepLast depth 5); line output/subscription
use volatile KeepLast depth 10. RMW endpoint discovery reports UNKNOWN history in this lab, so the
sensor check reports that honestly. The declared policy, successful intra-process construction and
actual callback flags verify compatibility. Public DDS topics remain available to mission, safety,
GCS and recorder subscribers, as exercised by the final GCS and bag checks.

Line observations/status, graph, selected path/candidates/status and obstacle output publish owned
messages with `std::unique_ptr`. The packaged RosGzBridge controls image allocation and exposes no
public ownership-selection hook; it was not forked. Actual image callback metadata confirms rclcpp
intra-process delivery. No end-to-end zero-copy claim is made: Gazebo conversion, RGB/BGR conversion
and external DDS consumers still have their normal work.

## Exact final commands and results

```bash
tools/lab_env.sh build
tools/lab_env.sh test
docker build -t line-follower-components:local .
tools/lab_env.sh measure-components
tools/lab_env.sh checkpoint-a
tools/lab_env.sh check-gcs
tools/lab_env.sh check-bag
```

All final commands exited **0**:

| Command | Key result |
|---|---|
| `build` | 8 packages finished, 1.82 s |
| `test` | 98 tests, 0 errors, 0 failures, 0 skipped |
| Docker build | 6 competition packages finished, 48.1 s; Release and BUILD_TESTING=OFF |
| `measure-components` | PASS; two 20 s RUNNING samples; actual image + line intra-process flags true |
| `checkpoint-a` | PASS; sensors 12/12, motion 14/14, controller loss 8/8, guard exit 7/7 |
| `check-gcs` | PASS, 26/26 |
| `check-bag` | PASS; real recording/Qt scrub/recomputed replay, no real velocity publishers/messages |

Logs: `log_lab/components-build.log`, `components-test.log`, `components-docker-build.log`,
`components-measure.log`, `components-checkpoint-a.log`, `components-check-gcs.log`,
`components-check-bag.log`. Checkpoint A was rerun alone after a concurrent run measured 22.56 Hz at
its external camera observer; the isolated final observer measured 27.25 Hz, while the detector's
own measured receipt rate is about 30.3 Hz. Both raw checkpoint logs are retained, including
`components-checkpoint-a-concurrent.log`; no measurement was rewritten or threshold relaxed.

The image tag is `line-follower-components:local` (image ID
`sha256:bda9518fcb1831e5b65ab8d8331b09b1ded5cdda7a8e7ae11775a3d56b76d4d1`).
The Dockerfile itself is unchanged and executed:

```text
colcon build --packages-up-to line_follower_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
Summary: 6 packages finished [48.1s]
```

## Before/after measurement

Both variants use the same refactored component code and YAML settings. The baseline selects the
generated standalone wrappers plus the external ros_gz_image bridge (`compose:=false`); the default
selects the generic image bridge and apps inside the container. Each sample lasts 20 wall seconds
while RUNNING, after the unchanged 20 s start gate and a 1 s warmup. Test speed is capped at 0.001 m/s
to keep the robot on the mat; normal competition.yaml is untouched. Maximum observed excursion over
the sampling window was about 0.021 m in each variant.

The observer deliberately has no raw-image subscription. Camera input seen by the detector is
estimated from its one-per-image-callback status stream; line output is independently counted.
CPU is process utime+stime over the window (100% = one CPU core), excluding Gazebo, the unchanged
rover parameter bridge, controls and observer. The baseline sums its three perception processes;
the composed number includes the image bridge and component manager in the one container process.

| Variant | Window (s) | Camera seen (Hz) | Line output (Hz) | Capture→publish p50 / p95 (ms) | Perception CPU (% of one core) |
|---|---:|---:|---:|---:|---:|
| standalone | 20.003 | 30.25 | 30.30 | 4.0 / 4.0 | 19.80 |
| composed | 20.000 | 30.30 | 30.30 | 4.0 / 8.0 | 22.40 |

Baseline CPU breakdown: image bridge 4.85%, line detector 11.55%, path planner 3.40%.
The final composed sample used 22.40%, an increase of 2.60 percentage points over the 19.80% baseline;
its measured p95 increased from 4 to 8 ms. Throughput was maintained, but these samples show no CPU
or latency gain. The latency field uses simulation-clock timestamps and is quantized by clock
updates; it is not a wall-clock transport trace. Both final samples had zero negative latency values.
The comparison also changes the bridge implementation, so effects are not attributed solely to IPC.

Raw final data: `log_lab/components/measurements.json`, with callback/output counts, stamp matches,
per-process CPU and owned PIDs. Earlier retained samples were baseline/composed 20.25/23.85% and
20.45/22.70%; they likewise do not establish a benefit. The first preliminary CLI inspection failed,
which is described below; its numeric sampling windows remain available for transparency. There is
no statistical/general performance claim for future non-dummy perception algorithms.

Actual callback evidence from the final composed run:

```text
PASS | actual intra-process callbacks | [component_container_mt-4] [INFO] [1795022686.158341786] [line_detector]: camera callback intra_process=true
[component_container_mt-4] [INFO] [1795022686.160850642] [path_planner]: line callback intra_process=true
```

## Component list and capture-stamp tests

In the running default competition graph, `ros2 component list --no-daemon --spin-time 3` reported:

```text
/perception_container
  1  /camera_bridge
  2  /line_detector
  3  /path_planner
```

This output is captured in the final sensor check below. The measurement additionally ran
`ros2 component list --no-daemon --spin-time 3 /perception_container` after its RUNNING sample.
The dynamic-factory C++ test constructs detector/planner/obstacle with both intra-process options
and checks the actual NodeBase setting. The fake-sensor launch test queries nondefault named YAML
parameters (camera_x_offset=0.123 and lookahead=0.42), lists the two app components, and verifies
camera stamps survive line, observed graph, valid selected path and graph-point capture stamps.
The existing topic-graph and safety assertions remain in place. Launch-policy tests guard camera
bridge exclusivity for true/True/false/False, force the extension off in competition, and keep all
Gazebo components out of replay. Standalone detector/planner wrappers were exercised by the actual
Gazebo baseline measurement; the obstacle factory and wrapper build were also verified.

## Final Checkpoint A output

```text
Summary: 8 packages finished [13.2s]
Summary: 98 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A test: PASS
PASS | /camera/image_raw              | 136 messages/5s; ['sensor_msgs/msg/Image']
PASS | /camera/camera_info            | 151 messages/5s; ['sensor_msgs/msg/CameraInfo']
PASS | /imu                           | 1001 messages/5s; ['sensor_msgs/msg/Imu']
PASS | /odom                          | 251 messages/5s; ['nav_msgs/msg/Odometry']
PASS | /clock                         | 1252 messages/5s; ['rosgraph_msgs/msg/Clock']
PASS | camera dimensions and rate     | 640x480; measured 27.25 Hz wall (world target 30 Hz)
PASS | velocity bridge subscriber     | [('checkpoint_a_sensors', 'geometry_msgs/msg/Twist'), ('ros_gz_bridge', 'geometry_msgs/msg/Twist')]
PASS | camera bridge publisher QoS    | [('camera_bridge', 'VOLATILE', 'UNKNOWN')]
PASS | competition component list     | exit=0; /perception_container
  1  /camera_bridge
  2  /line_detector
  3  /path_planner; 
PASS | no scoped bridge interfaces    | ROS graph hits=[]; bridge config scanned
PASS | provided asset SHA256          | 12 files verified; 
PASS | owned process group cleaned    | live PIDs=[]
sensors: PASS (12/12 checks)
CHECKPOINT A check-sensors: PASS
PASS | forward                        | dx=+0.0694m dy=+0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after forward             | stop=0.040s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | backward                       | dx=-0.0713m dy=-0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after backward            | stop=0.084s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | left turn                      | dx=+0.0454m dy=+0.0146m yaw=+0.6140rad; peak=0.040/0.500
PASS | stop after left turn           | stop=0.056s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | right turn                     | dx=+0.0443m dy=-0.0139m yaw=-0.5980rad; peak=0.040/0.500
PASS | stop after right turn          | stop=0.044s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate left                    | dx=-0.0000m dy=-0.0000m yaw=+0.7200rad; peak=0.000/0.600
PASS | stop after rotate left         | stop=0.055s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate right                   | dx=-0.0000m dy=+0.0000m yaw=-0.7104rad; peak=0.000/0.600
PASS | stop after rotate right        | stop=0.044s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | bounded excursion              | max from spawn odom=0.0920m
PASS | owned process group cleaned    | live PIDs=[]
motion: PASS (14/14 checks)
CHECKPOINT A check-motion: PASS
PASS | 20 second startup gate         | elapsed=20.204s; launch wall=20.817s
PASS | all pre-RUNNING commands zero  | 1006 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 860 samples; displacement=0.000000m
PASS | dummy pipeline drives          | odom speed=0.0294m/s
PASS | controller loss command stop   | PID=50282; zero after 0.178s; allowed=0.300s
PASS | controller loss physical stop  | stop after 0.233s; final speed=(7.762887554996212e-15, 1.1816670367716578e-13)
PASS | safety explains stop           | ['', 'command candidate stale']
PASS | owned process group cleaned    | live PIDs=[]
start-stop: PASS (8/8 checks)
CHECKPOINT A check-start-stop: PASS
PASS | 20 second startup gate         | elapsed=20.204s; launch wall=20.815s
PASS | all pre-RUNNING commands zero  | 1006 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 847 samples; displacement=0.000000m
PASS | dummy pipeline drives          | odom speed=0.0294m/s
PASS | safety_guard exit leaves zero  | PID=50504; post-exit messages=6; last_zero=True
PASS | safety_guard exit physical stop | stop after 0.056s; final speed=(1.0733601507606494e-15, 7.423196059003542e-14)
PASS | owned process group cleaned    | live PIDs=[]
guard-exit: PASS (7/7 checks)
CHECKPOINT A check-guard-exit: PASS
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
```

The D-02 check now starts the composed competition graph with only test speed set to zero, checks
the composed camera publisher/QoS and lists the components. The original <5 Hz failure threshold
is unchanged. Its external Python observer measured 27.25 Hz; the no-image-observer benchmark above
measured about 30.3 detector callbacks/s. The bridge configuration contains no scoped-world ROS
interfaces, and all 12 provided asset hashes were verified.

## External DDS and replay acceptance

Final GCS output:

```text
.........                                                                [100%]
9 passed in 0.06s
PASS | START before READY rejected    | rejected: mission not READY
PASS | wrong run rejected             | run_id mismatch
PASS | bag START missing recorder     | timeout: service unavailable or response deadline exceeded
PASS | bag STOP missing recorder      | timeout: service unavailable or response deadline exceeded
PASS | GCS snapshot rate              | 6 samples; 1.999 Hz
PASS | snapshot relay state equality  | 6 matched stamps; max relay=0.0011s
PASS | odom geometry and provenance   | graph=1 path=10 sources=[0]
PASS | link loss visible and isolated | LOST after 2.047s; all five rover nodes alive; mission publishes
PASS | link recovers                  | OK
PASS | START READY manual accepted    | start requested
PASS | duplicate returns original     | accepted=True; cached state=2; same response=True
PASS | START RUNNING rejected         | rejected: mission not READY
PASS | live RUNNING reaches UI feed   | GCS state=3
PASS | RUNNING display within period  | rover→GCS state delay=0.3016s (2 Hz + 50ms transport margin)
PASS | real rover moves               | odom speed=0.1000m/s
PASS | UI exit does not stop rover    | UI PID=47977; rover alive, mission heartbeat, speed=0.1000
PASS | KILL active accepted           | accepted: kill requested (software stop, not a physical E-STOP)
PASS | KILL E_STOP then STOPPED       | states=[0, 1, 2, 3, 7, 8]
PASS | KILL final velocity zero       | cmd=0.0; odom=0.000000
PASS | KILL inactive rejected         | rejected: mission already inactive
PASS | timeline retains short states  | GCS transition events=[1, 2, 3, 7, 8]
PASS | live Qt map/camera render      | last render=('36', '83', '3')
PASS | live UI reception              | snapshots/camera/elapsed/Hz=('36', '84', '19.99', '1.80', '4.20')
PASS | GCS processes healthy          | []
PASS | stopped sources marked stale   | stale_sources=['candidates', 'observed_graph', 'path_status', 'perception_status', 'safety_status', 'selected_path']; odom remains fresh
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-gcs-xsksqhli/launch.log
GCS D-11/D-12: PASS (26/26)

```

Planner/perception source-loss injection now unloads those components through their manager's
ListNodes/UnloadNode services; safety_guard remains an owned standalone PID. This preserves the
source-loss test without killing unrelated components or assuming a PID per logical node.
The GCS receives the camera and public perception/path/status snapshots, and UI-only exit leaves
the rover running. The component container did not require a dependency on GCS.

Key final bag/replay output (complete topic counts and metadata are in the full log):

```text
PASS | STOP while idle rejected       | not recording
PASS | recorder wrong run rejected    | run_id mismatch
PASS | record START accepted          | /ws/bags/check-stage2/433166dc/20261118T172942_ca5d9133
PASS | duplicate START original reply | /ws/bags/check-stage2/433166dc/20261118T172942_ca5d9133
PASS | different command ID rejected  | request_id reused with different command
PASS | second START rejected          | already recording
PASS | recording status active        | state=RECORDING
PASS | recorded KILL accepted         | accepted: kill requested (software stop, not a physical E-STOP)
PASS | recorded rover STOPPED         | state=8
PASS | record STOP closes cleanly     | stop requested; close=0.165s
PASS | duplicate STOP original reply  | stop requested
PASS | second STOP rejected           | not recording
PASS | recording status STOPPED       | STOPPED
PASS | expected topics and counts     | missing=[]; counts={"/camera/camera_info": 548, "/camera/image_raw": 452, "/clock": 4527, "/cmd_vel": 903, "/control/cmd_vel_candidate": 544, "/gcs/command_events": 1, "/gcs/recording_status": 91, "/imu": 3622, "/localization/estimate": 898, "/mission/gcs_snapshot": 36, "/mission/state": 191, "/odom": 906, "/path/candidates": 543, "/path/local": 543, "/path/observed_graph": 543, "/path/status": 543, "/perception/line": 544, "/perception/status": 544, "/safety/status": 902}
PASS | full mission event sequence    | ['STARTUP_WAIT', 'CHECKING', 'READY', 'RUNNING', 'E_STOP', 'STOPPED']
PASS | KILL command event             | 12 total events
EVENT 0.108s mission: STARTUP_WAIT
EVENT 0.156s perception: healthy=True fallback=False 
EVENT 0.172s safety: drive not enabled
EVENT 0.188s mode: VISION_TRACKING
EVENT 16.400s mission: CHECKING
EVENT 16.500s mission: READY
EVENT 16.600s mission: RUNNING
EVENT 16.616s safety: command approved
EVENT 16.964s command: kill: accepted kill requested (software stop, not a physical E-STOP)
EVENT 17.000s mission: E_STOP
EVENT 17.016s safety: drive not enabled
EVENT 17.100s mission: STOPPED
PASS | CLI uses shared extractor      | JSON=/ws/bags/check-stage2/433166dc/analysis.json; exit=0; stderr=
PASS | bounded recording artifact     | bag=/ws/bags/check-stage2/433166dc/20261118T172942_ca5d9133; total=9.08 MiB; duration=18.104s
PASS | UI bag RUNNING scrub | t=16.901s; map observed/planned; camera=640x480; commands disabled
PASS | UI bag end scrub | state=STOPPED; events=12; same shared extractor
PASS | UI LIVE return | replay ended; seek disabled
PASS | real bag offscreen scrub       | exit=0; stderr=QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-ubuntu'
PASS | owned process group cleaned    | live PIDs=[]
PASS | replay never publishes cmd_vel | max publishers=0; received=0
PASS | replay command redirected      | /replay/cmd_vel_blocked count=587; nonzero=447
PASS | perception/path regenerated    | {"/path/local": 492, "/path/status": 492, "/perception/line": 450, "/perception/status": 450}
PASS | replay excludes Gazebo bridge  | nodes=['checkpoint_a_bag_replay', 'launch_ros_48788', 'line_detector', 'mission_manager', 'motion_controller', 'path_planner', 'perception_container', 'safety_guard']
PASS | owned process group cleaned    | live PIDs=[]
D-13 RECORD/REPLAY: PASS; artifacts=/ws/bags/check-stage2/433166dc
```

The bag contains real camera/perception/path/control data and KILL events; replay regenerates the
pipeline using the two-app default container, without any Gazebo camera bridge. There were zero
/cmd_vel publishers and zero received messages, with 587 blocked commands (447 nonzero), 450 line
and perception-status messages, and 492 local-path/path-status messages in the final replay.
The recording is retained under `bags/check-stage2/433166dc/` and is bounded/ignored as before.
This run supersedes the previous task's retained check-bag sample.

## Earlier development failures

These are not counted as final failures, per the coordinator's resume instruction; they are not hidden:

- Before registration, the new factory test failed with `Could not find requested resource in ament index`
  (`log_lab/components-factories-red.log`), then passed after conversion.
- A development compile used a nonexistent MessageInfo accessor, `get_from_intra_process`; Jazzy's
  actual accessor is `get_rmw_message_info().from_intra_process`, now used and compiled successfully.
- The first expanded pipeline test queried ListNodes as soon as the service existed and got an empty
  list while components were still loading. `components-test-readiness-red.log` records the failure
  (Ctest/JUnit reported two failure entries for that one assertion). It now waits for the exact loaded
  set, and the capture test runs after the original startup lifecycle test.
- A preliminary measurement helper's default-discovery CLI probe produced no list. The numeric
  RUNNING samples and actual intra-process callbacks already worked; an explicit 3 s discovery
  inspection succeeded, and the helper was rerun successfully. See
  `components-measure-discovery-initial.log`, `components/measurements-initial.json` and the successful
  `components-list-probe.log`; final measurement and before-resume data are retained separately.
- A temporary inline probe overwrote PYTHONPATH and could not import rclpy; preserving the ROS setup's
  PYTHONPATH fixed that probe. This was not an application/launch defect.
- Test-first checks exposed capitalized True selecting the wrong external-bridge branch, and absent
  explicit competition/replay exclusion guards. They were fixed before the final runs; red outputs
  are `components-bridge-selection-red.log` and `components-profile-policy-red.log`.

No final Gazebo acceptance run failed. An earlier concurrent checkpoint run also passed, with a lower
22.56 Hz external camera reading; the final isolated checkpoint result is reported above.

## Production closure, unchanged inputs and cleanup

`docker exec line_follower_lab bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; colcon list --packages-up-to line_follower_bringup --names-only'`:

```text
line_follower_bringup
line_follower_control
line_follower_interfaces
line_follower_mission
line_follower_navigation
line_follower_perception

```

Neither line_follower_gcs nor Qt enters this package closure. Application package manifests now
correctly depend on rclcpp_components; bringup declares the container runtime and retains its existing
ros_gz_bridge dependency. Provided worlds/models/SHA256SUMS, normal competition parameters, message
interfaces and design §17 checkboxes were not changed by this task. No git repository was initialized.

Final `docker exec line_follower_lab ps -eo pid,pgid,args`:

```text
    PID    PGID COMMAND
      1       1 /sbin/docker-init -- bash -c mkdir -p "$HOME" && exec sleep infinity
     57      57 sleep infinity
  50690   50690 ps -eo pid,pgid,args

```

Only container init, idle sleep and the inspection command remain. No Gazebo, launch, component,
GCS, recorder or player process is left. Cleanup used only owned process groups; no uav_ugv_* or
line_follower_verify container was touched.

## Files added or changed

- `README.md`
- `docs/test_plan.md`
- `docs/components_evidence.md`
- `src/line_follower_perception/CMakeLists.txt`
- `src/line_follower_perception/package.xml`
- `src/line_follower_perception/src/line_detector_node.cpp`
- `src/line_follower_perception/src/obstacle_detector_node.cpp`
- `src/line_follower_navigation/CMakeLists.txt`
- `src/line_follower_navigation/package.xml`
- `src/line_follower_navigation/src/path_planner_node.cpp`
- `src/line_follower_bringup/package.xml`
- `src/line_follower_bringup/config/camera_bridge.yaml`
- `src/line_follower_bringup/launch/nodes.launch.py`
- `src/line_follower_bringup/launch/sim.launch.py`
- `src/line_follower_bringup/launch/competition.launch.py`
- `src/line_follower_bringup/launch/dev_sim.launch.py`
- `src/line_follower_bringup/launch/replay.launch.py`
- `src/line_follower_tests/CMakeLists.txt`
- `src/line_follower_tests/package.xml`
- `src/line_follower_tests/test/test_component_factories.cpp`
- `src/line_follower_tests/test/test_component_launch.py`
- `src/line_follower_tests/test/test_pipeline_launch.py`
- `src/line_follower_tests/test/test_replay_policy.py`
- `tools/lab_env.sh`
- `tools/checkpoint_a.py`
- `tools/check_gcs.py`
- `tools/check_bag.py`
- `tools/measure_components.py`

28 source/documentation files; generated build/install/log and bounded bag artifacts are excluded.

## Limits

No required item is outstanding; independent lead verification remains. The full three-component
variant works, so no fallback was shipped. The app algorithms remain dummies; these are construction,
communication, timestamp, simulation/safety and measured-workload results, not a line-following or
future heavy-inference performance claim. External DDS image delivery is best-effort and its observer
rate is reported explicitly. No image-allocation/copy-count trace or hard real-time guarantee is
claimed. There were no screenshot features, provided asset edits, or unrelated project actions.
