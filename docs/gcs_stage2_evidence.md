# GCS stage 2 implementation and evidence

Date: 2026-11-18. Workspace: `/home/sion/Line_Tracer /line_follower_ws`.

**Dispatch outcome under the requested “any non-zero check = FAILED” rule: FAILED.**
The initial real safety_guard-exit check failed, and one intermediate bag/UI acceptance run failed.
Both were repaired; **every final acceptance command exits 0**, and all requested implementation
work is complete. The failures are preserved below rather than hidden by the final green reruns.
Independent lead verification is the remaining step for accepting the remediation.

## Initial failures and remediation

1. `tools/lab_env.sh check-guard-exit` initially exited **1**:

```text
PASS | 20 second startup gate         | elapsed=20.207s; launch wall=20.906s
PASS | all pre-RUNNING commands zero  | 1006 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 854 samples; displacement=0.000000m
PASS | dummy pipeline drives          | odom speed=0.0314m/s
FAIL | safety_guard exit leaves zero  | PID=21535; post-exit messages=3; last_zero=False
FAIL | safety_guard exit physical stop | stop after infs; final speed=(0.06999999999990347, 2.659506374319844e-13)
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-guard-exit-16yiv0mo/launch.log
guard-exit: FAIL (5/7 checks)

```

This was a real safety failure: the robot retained a nonzero command and continued at 0.070 m/s.
It was reported to the coordinator in question `msg_dfbf4222614e`. The old pre-shutdown callback
ran in ROS's dedicated signal thread while the executor/timer could still publish another driving
command. The fix removes that race: a minimal SIGINT/SIGTERM flag ends the single-threaded executor,
then cancels the timer, publishes five zeros while the ROS context is still valid, waits for reliable
DDS acknowledgements, and only then shuts down. No zero-command or physical-stop assertion was relaxed.
The final isolated exit check received five post-interrupt commands, left the last exactly zero, and stopped odom in 0.059 s.

Final guard log acknowledgement (`log_lab/stage2-guard-dds-ack.log`):

```text
[safety_guard-8] [INFO] [1795007424.000832967] [safety_guard]: shutdown zero published (DDS acknowledged=true)

```

2. An intermediate `tools/lab_env.sh check-bag` exited **1**, preserved in
`log_lab/stage2-check-bag-ui-rounding-failed.log`. Its offscreen UI check timed out waiting for
RUNNING: the 1000-position slider rounded a requested 13.717 s to 13.71148 s, before the first
RUNNING snapshot (a diagnostic seek at 13.7264 s did show RUNNING). The final UI uses millisecond
positions and exact event timestamps; its reader also refines state and pose from raw mission/odom
samples instead of retaining a slower snapshot's state. A regression test first reproduced the
wrong state and now passes, including protection against mutating cached snapshots. Scrubbing to
before the first sample clears the state/camera instead of leaving a later sample displayed.
Failed check bags are removed on the next completed run to bound disk usage; their failed logs remain.

Other development red states were expected TDD unit failures: missing bag primitives, stale-layer
handling, millisecond seek behavior, wrapped headings for already-odom paths, and the new stop-grace
argument. These were fixed before final acceptance reruns.

## Final commands and results

All ROS/Gazebo commands ran inside `line_follower_lab` through the lab wrapper; no host ROS was used.

```bash
tools/lab_env.sh build
tools/lab_env.sh test
tools/lab_env.sh checkpoint-a
tools/lab_env.sh check-gcs
tools/lab_env.sh check-bag
```

All final commands exited **0**:

- Build: `Summary: 8 packages finished [24.5s]`.
- Standalone test: `Summary: 85 tests, 0 errors, 0 failures, 0 skipped`.
- Checkpoint A: all four runtime checks plus colcon test PASS.
- GCS stage 1 and review fixes: **26/26 PASS**.
- D-13 real recording, event analysis, UI scrub and pipeline replay: **PASS**.

Full logs: `log_lab/stage2-build.log`, `stage2-test.log`, `stage2-checkpoint-a.log`,
`stage2-check-gcs.log`, `stage2-check-bag.log`. The final system checks were run with isolated domains
and unique Gazebo partitions; concurrent checks do not share a ROS/Gazebo graph.

## Updated Checkpoint A tables

```text
Summary: 8 packages finished [11.6s]
Summary: 85 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A test: PASS
PASS | /camera/image_raw              | 134 messages/5s; ['sensor_msgs/msg/Image']
PASS | /camera/camera_info            | 151 messages/5s; ['sensor_msgs/msg/CameraInfo']
PASS | /imu                           | 1000 messages/5s; ['sensor_msgs/msg/Imu']
PASS | /odom                          | 250 messages/5s; ['nav_msgs/msg/Odometry']
PASS | /clock                         | 1251 messages/5s; ['rosgraph_msgs/msg/Clock']
PASS | camera dimensions and rate     | 640x480; measured 26.84 Hz wall (world target 30 Hz)
PASS | velocity bridge subscriber     | [('checkpoint_a_sensors', 'geometry_msgs/msg/Twist'), ('ros_gz_bridge', 'geometry_msgs/msg/Twist')]
PASS | no scoped bridge interfaces    | ROS graph hits=[]; bridge config scanned
PASS | provided asset SHA256          | 12 files verified; 
PASS | owned process group cleaned    | live PIDs=[]
sensors: PASS (10/10 checks)
CHECKPOINT A check-sensors: PASS
PASS | forward                        | dx=+0.0701m dy=+0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after forward             | stop=0.051s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | backward                       | dx=-0.0725m dy=-0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after backward            | stop=0.084s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | left turn                      | dx=+0.0452m dy=+0.0145m yaw=+0.6120rad; peak=0.040/0.500
PASS | stop after left turn           | stop=0.045s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | right turn                     | dx=+0.0446m dy=-0.0141m yaw=-0.6020rad; peak=0.040/0.500
PASS | stop after right turn          | stop=0.056s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate left                    | dx=-0.0000m dy=-0.0000m yaw=+0.7224rad; peak=0.000/0.600
PASS | stop after rotate left         | stop=0.056s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate right                   | dx=-0.0000m dy=+0.0000m yaw=-0.7128rad; peak=0.000/0.600
PASS | stop after rotate right        | stop=0.060s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | bounded excursion              | max from spawn odom=0.0919m
PASS | owned process group cleaned    | live PIDs=[]
motion: PASS (14/14 checks)
CHECKPOINT A check-motion: PASS
PASS | 20 second startup gate         | elapsed=20.205s; launch wall=20.730s
PASS | all pre-RUNNING commands zero  | 1006 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 830 samples; displacement=0.000000m
PASS | dummy pipeline drives          | odom speed=0.0314m/s
PASS | controller loss command stop   | PID=37890; zero after 0.197s; allowed=0.300s
PASS | controller loss physical stop  | stop after 0.255s; final speed=(9.454242944073599e-15, 9.489602290186714e-14)
PASS | safety explains stop           | ['', 'command candidate stale']
PASS | owned process group cleaned    | live PIDs=[]
start-stop: PASS (8/8 checks)
CHECKPOINT A check-start-stop: PASS
PASS | 20 second startup gate         | elapsed=20.205s; launch wall=20.747s
PASS | all pre-RUNNING commands zero  | 840 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 833 samples; displacement=0.000000m
PASS | dummy pipeline drives          | odom speed=0.0334m/s
PASS | safety_guard exit leaves zero  | PID=38123; post-exit messages=5; last_zero=True
PASS | safety_guard exit physical stop | stop after 0.059s; final speed=(8.120674271916428e-15, 9.096720666707647e-14)
PASS | owned process group cleaned    | live PIDs=[]
guard-exit: PASS (7/7 checks)
CHECKPOINT A check-guard-exit: PASS
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
```

Follow-ups 1–5 are covered: isolated safety_guard SIGINT, immediate controller-loss bound
`candidate_timeout + 2/control_rate + 0.06` = 0.300 s, straight-command lateral/yaw negative cases,
100 ms polling for up to 1 s after owned-group SIGKILL (including reaping), and a 3 s retry for owned
PID discovery with unbuffered launch output. SIGINT targets only a PID identified in that launch's
log and verified to belong to its process group.

## Stage-1 independent-review fixes

```text
.........                                                                [100%]
9 passed in 0.07s
PASS | START before READY rejected    | rejected: mission not READY
PASS | wrong run rejected             | run_id mismatch
PASS | bag START missing recorder     | timeout: service unavailable or response deadline exceeded
PASS | bag STOP missing recorder      | timeout: service unavailable or response deadline exceeded
PASS | GCS snapshot rate              | 6 samples; 2.000 Hz
PASS | snapshot relay state equality  | 6 matched stamps; max relay=0.0009s
PASS | odom geometry and provenance   | graph=1 path=10 sources=[0]
PASS | link loss visible and isolated | LOST after 2.096s; all five rover nodes alive; mission publishes
PASS | link recovers                  | OK
PASS | START READY manual accepted    | start requested
PASS | duplicate returns original     | accepted=True; cached state=2; same response=True
PASS | START RUNNING rejected         | rejected: mission not READY
PASS | live RUNNING reaches UI feed   | GCS state=3
PASS | RUNNING display within period  | rover→GCS state delay=0.3016s (2 Hz + 50ms transport margin)
PASS | real rover moves               | odom speed=0.1000m/s
PASS | UI exit does not stop rover    | UI PID=36875; rover alive, mission heartbeat, speed=0.1000
PASS | KILL active accepted           | accepted: kill requested (software stop, not a physical E-STOP)
PASS | KILL E_STOP then STOPPED       | states=[0, 1, 2, 3, 7, 8]
PASS | KILL final velocity zero       | cmd=0.0; odom=0.000000
PASS | KILL inactive rejected         | rejected: mission already inactive
PASS | timeline retains short states  | GCS transition events=[1, 2, 3, 7, 8]
PASS | live Qt map/camera render      | last render=('36', '83', '3')
PASS | live UI reception              | snapshots/camera/elapsed/Hz=('36', '84', '19.92', '1.81', '4.22')
PASS | GCS processes healthy          | []
PASS | stopped sources marked stale   | stale_sources=['candidates', 'observed_graph', 'path_status', 'perception_status', 'safety_status', 'selected_path']; odom remains fresh
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-gcs-slk18ur7/launch.log
GCS D-11/D-12: PASS (26/26)

```

The default `dev_sim` no longer stops the rover when the UI exits; `stop_on_gcs_exit:=true` is an
explicit convenience option used by managed `tools/lab_env.sh gcs` sessions. The check terminated
only the UI while RUNNING and observed all five rover nodes, mission heartbeats and continuing drive.

Snapshots now carry perception status and wall-time `stale_sources` for pose, graph, candidates,
selected path and path/safety/perception status. The check stopped planner/perception/safety publishers
and verified six stale sources while odometry remained fresh. The adapter hides stale geometry and
the window marks stale status. Candidate rule/MLP scores and selection are preserved and rendered
through width/alpha/labels with NaN-safe MLP text; pure/offscreen tests cover those paths. LOST is red,
STALE amber. Request IDs reused for a different command are rejected, and response-cache eviction
cannot re-execute an old ID because a per-run seen-ID map remains. Tests verify both cases.

The managed stop path escalates only an identity-checked owned group to SIGKILL after its grace period;
a node-free process test exercised escalation and verified that a mismatched PID identity is not
signalled. `target_heading` is normalized for both converted and already-odom paths, and the missing
`<stdexcept>` include is present. The accepted four grid spans and 1600×1000 default window are unchanged.
No change was needed for the previously reviewed main-thread Qt signal connection.

## D-13 recording, metadata, events, UI replay and pipeline replay

Final container bag path:

```text
/ws/bags/check-stage2/2a2d3562/20261118T130848_9bcf0c34
```

The same host-relative directory is under `bags/check-stage2/`; its shared analyzer output is
`bags/check-stage2/2a2d3562/analysis.json`. Bags and logs are gitignored/dockerignored. The final artifact is
10.32 MiB including its adjacent log/JSON, with a 17.572 s bag duration; one completed check directory
is retained. Normal recorder guards are 300 wall seconds and 1 GiB. Recording uses compressed MCAP
(Zstd chunks) and closes by SIGINT to its own child process group.

Complete final D-13 output:

```text
PASS | STOP while idle rejected       | not recording
PASS | recorder wrong run rejected    | run_id mismatch
PASS | record START accepted          | /ws/bags/check-stage2/2a2d3562/20261118T130848_9bcf0c34
PASS | duplicate START original reply | /ws/bags/check-stage2/2a2d3562/20261118T130848_9bcf0c34
PASS | different command ID rejected  | request_id reused with different command
PASS | second START rejected          | already recording
PASS | recording status active        | state=RECORDING
PASS | recorded KILL accepted         | accepted: kill requested (software stop, not a physical E-STOP)
PASS | recorded rover STOPPED         | state=8
PASS | record STOP closes cleanly     | stop requested; close=0.216s
PASS | duplicate STOP original reply  | stop requested
PASS | second STOP rejected           | not recording
PASS | recording status STOPPED       | STOPPED
PASS | expected topics and counts     | missing=[]; counts={"/camera/camera_info": 530, "/camera/image_raw": 529, "/clock": 4394, "/cmd_vel": 883, "/control/cmd_vel_candidate": 450, "/gcs/command_events": 1, "/gcs/recording_status": 89, "/imu": 3513, "/localization/estimate": 879, "/mission/gcs_snapshot": 36, "/mission/state": 188, "/odom": 874, "/path/candidates": 450, "/path/local": 449, "/path/observed_graph": 450, "/path/status": 449, "/perception/line": 447, "/perception/status": 447, "/safety/status": 883}
PASS | full mission event sequence    | ['STARTUP_WAIT', 'CHECKING', 'READY', 'RUNNING', 'E_STOP', 'STOPPED']
PASS | KILL command event             | 12 total events
EVENT 0.016s mission: STARTUP_WAIT
EVENT 0.044s mode: VISION_TRACKING
EVENT 0.108s perception: healthy=True fallback=False 
EVENT 0.120s safety: drive not enabled
EVENT 15.964s mission: CHECKING
EVENT 16.064s mission: READY
EVENT 16.164s mission: RUNNING
EVENT 16.164s safety: command approved
EVENT 16.560s command: kill: accepted kill requested (software stop, not a physical E-STOP)
EVENT 16.564s mission: E_STOP
EVENT 16.564s safety: drive not enabled
EVENT 16.664s mission: STOPPED
PASS | CLI uses shared extractor      | JSON=/ws/bags/check-stage2/2a2d3562/analysis.json; exit=0; stderr=
PASS | bounded recording artifact     | bag=/ws/bags/check-stage2/2a2d3562/20261118T130848_9bcf0c34; total=10.32 MiB; duration=17.572s
PASS | UI bag RUNNING scrub | t=16.465s; map observed/planned; camera=640x480; commands disabled
PASS | UI bag end scrub | state=STOPPED; events=12; same shared extractor
PASS | UI LIVE return | replay ended; seek disabled
PASS | real bag offscreen scrub       | exit=0; stderr=QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-ubuntu'
This plugin does not support propagateSizeHints()
This plugin does not support propagateSizeHints()
This plugin does not support propagateSizeHints()
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-gcs-37gz8ypx/launch.log
PASS | replay never publishes cmd_vel | max publishers=0; received=0
PASS | replay command redirected      | /replay/cmd_vel_blocked count=579; nonzero=429
PASS | perception/path regenerated    | {"/path/local": 563, "/path/status": 563, "/perception/line": 512, "/perception/status": 512}
PASS | replay excludes Gazebo bridge  | nodes=['checkpoint_a_bag_replay', 'line_detector', 'mission_manager', 'motion_controller', 'path_planner', 'safety_guard']
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-bag-replay-s4dfv7zn/launch.log
D-13 RECORD/REPLAY: PASS; artifacts=/ws/bags/check-stage2/2a2d3562
```

Topic counts in the final bag:

| Topic | Messages |
|---|---:|
| `/camera/camera_info` | 530 |
| `/camera/image_raw` | 529 |
| `/clock` | 4394 |
| `/cmd_vel` | 883 |
| `/control/cmd_vel_candidate` | 450 |
| `/gcs/command_events` | 1 |
| `/gcs/recording_status` | 89 |
| `/imu` | 3513 |
| `/localization/estimate` | 879 |
| `/mission/gcs_snapshot` | 36 |
| `/mission/state` | 188 |
| `/odom` | 874 |
| `/path/candidates` | 450 |
| `/path/local` | 449 |
| `/path/observed_graph` | 450 |
| `/path/status` | 449 |
| `/perception/line` | 447 |
| `/perception/status` | 447 |
| `/safety/status` | 883 |

All configured active topics have messages. `/perception/obstacles` is intentionally absent because
the extension publisher is disabled. The bag actually contains velocity and KILL command data, so
replay isolation was tested against real command-bearing input rather than a sensor-only fixture.
The single source of recording topics is `config/record_topics.json`, also exposed as the bringup
launch's `RECORD_TOPICS` constant.

Final `run_metadata.json` (the check recomputed and matched its source/config SHA):

```json
{
  "closed_cleanly": true,
  "code_config_sha256": "896962c8883f3889db5acc0488899cb2d4cc8564dd1107d6121cb1f577d00a2d",
  "environment": {
    "compression": "zstd_fast",
    "gz_partition": "bag_record_36491",
    "hostname": "9704990b1686",
    "platform": "Linux-7.0.0-42-generic-x86_64-with-glibc2.39",
    "ros_distro": "jazzy",
    "ros_domain_id": "84",
    "storage": "mcap"
  },
  "params_file": "/ws/install_lab/line_follower_bringup/share/line_follower_bringup/config/competition.yaml",
  "recorder_exit_code": 0,
  "recorder_requests": [
    {
      "accepted": true,
      "command": 0,
      "reason": "/ws/bags/check-stage2/2a2d3562/20261118T130848_9bcf0c34",
      "request_id": "6ab31f8d-174e-458b-b5cf-5e22d4562ed7",
      "sent_stamp_ns": 628000000
    },
    {
      "accepted": false,
      "command": 0,
      "reason": "already recording",
      "request_id": "de61b813-4052-4669-b8fc-6f2a6d5e9b34",
      "sent_stamp_ns": 640000000
    },
    {
      "accepted": true,
      "command": 1,
      "reason": "stop requested",
      "request_id": "e3e1e02a-a0eb-4513-bc8c-988294ad5bf6",
      "sent_stamp_ns": 18720000000
    },
    {
      "accepted": false,
      "command": 1,
      "reason": "not recording",
      "request_id": "a1c4c4bb-301d-420c-a9cd-e7a36c1a3007",
      "sent_stamp_ns": 18940000000
    }
  ],
  "run_id": "run_20261118_130845_813172613463576",
  "schema_version": 1,
  "start": {
    "ros_ns": 628000000,
    "utc": "2026-11-18T13:08:48.957164+00:00",
    "wall_monotonic": 813176.04733662
  },
  "stop": {
    "ros_ns": 18720000000,
    "utc": "2026-11-18T13:09:07.452727+00:00",
    "wall_monotonic": 813194.542903889
  },
  "stop_reason": "stop requested",
  "topics": [
    "/camera/image_raw",
    "/camera/camera_info",
    "/clock",
    "/imu",
    "/odom",
    "/localization/estimate",
    "/perception/line",
    "/perception/status",
    "/path/observed_graph",
    "/path/local",
    "/path/candidates",
    "/path/status",
    "/control/cmd_vel_candidate",
    "/cmd_vel",
    "/mission/state",
    "/safety/status",
    "/perception/obstacles",
    "/mission/gcs_snapshot",
    "/gcs/command_events",
    "/gcs/recording_status"
  ],
  "use_sim_time": true
}
```

Event times are relative to the bag's first simulation-time timestamp; the mission startup gate uses
wall elapsed time. Thus RUNNING need not be at bag-relative 20 s even though the unchanged 20 s wall
gate is separately verified above. The extractor produces the complete STARTUP_WAIT→CHECKING→READY→
RUNNING→E_STOP→STOPPED sequence plus the accepted KILL result, mode, perception health/fallback and
safety changes. Every event carries its run ID; the UI loads the same event list as the CLI.

UI replay was tested through the real reader worker with this bag: map/state at RUNNING, a 640×480
camera frame, STOPPED at the end, event markers, disabled command buttons, and return to LIVE.
Pipeline replay is separate: the existing `replay.launch.py` sensor-only allowlist and final remap
were retained. The check observed **0 `/cmd_vel` publishers and 0 messages throughout**, **579 blocked
commands (429 nonzero)**, and regenerated **512 perception line/status messages** and **563 local
path/status messages**, with no Gazebo bridge. A static positive-allowlist test also guards against
playing any command topic.

Useful user commands:

```bash
tools/lab_env.sh gcs
# Use Bag START / STOP, then OPEN BAG / REPLAY and the millisecond seek slider.
# In a sourced lab shell:
python3 tools/bag_analyzer/run_analyzer.py /ws/bags/<bag>
python3 tools/bag_analyzer/run_analyzer.py /ws/bags/<bag> --json --output /ws/bags/events.json
ros2 launch line_follower_bringup replay.launch.py bag:=/ws/bags/<bag> rate:=2.0
```

## Competition independence and cleanup

`docker exec line_follower_lab bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; colcon list --packages-up-to line_follower_bringup --names-only'`:

```text
line_follower_bringup
line_follower_control
line_follower_interfaces
line_follower_mission
line_follower_navigation
line_follower_perception

```

Neither GCS, Qt nor the recorder enters this closure. The competition one-line launch, its package
manifest, provided assets/manifest and autonomous strategy are unchanged; the required clean-exit
safety defect was repaired. Design §17 boxes were not edited by this worker. Only dev-only package
runtime dependencies were extended for the recorder/reader.

Final `docker exec line_follower_lab ps -eo pid,pgid,args`:

```text
    PID    PGID COMMAND
      1       1 /sbin/docker-init -- bash -c mkdir -p "$HOME" && exec sleep infinity
     57      57 sleep infinity
  38646   38646 ps -eo pid,pgid,args

```

No Gazebo, launch, GCS, recorder, player or rover process remains. No other project's container or
process group, including `line_follower_verify` and `uav_ugv_*`, was stopped.

## Files added or changed

Generated build/install/log files and the bounded bag artifact are excluded from this source list.

- `.gitignore`
- `.dockerignore`
- `README.md`
- `docs/test_plan.md`
- `docs/gcs_stage2_evidence.md`
- `tools/lab_env.sh`
- `tools/checkpoint_a.py`
- `tools/check_gcs.py`
- `tools/check_bag.py`
- `tools/check_bag_ui.py`
- `tools/gcs_session.py`
- `tools/bag_analyzer/run_analyzer.py`
- `tools/gcs/setup.py`
- `tools/gcs/package.xml`
- `tools/gcs/line_follower_gcs/gcs_command.py`
- `tools/gcs/line_follower_gcs/gcs_mission_manager.py`
- `tools/gcs/line_follower_gcs/gcs_recorder.py`
- `tools/gcs/line_follower_gcs/bag_logic.py`
- `tools/gcs/line_follower_gcs/bag_archive.py`
- `tools/gcs/line_follower_gcs/replay_worker.py`
- `tools/gcs/line_follower_gcs/adapter.py`
- `tools/gcs/line_follower_gcs/window.py`
- `tools/gcs/test/test_bag_logic.py`
- `tools/gcs/test/test_bag_archive.py`
- `tools/gcs/test/test_fingerprint.py`
- `tools/gcs/test/test_adapter.py`
- `tools/gcs/test/test_window.py`
- `src/line_follower_control/src/safety_guard_node.cpp`
- `src/line_follower_interfaces/CMakeLists.txt`
- `src/line_follower_interfaces/msg/RecordingStatus.msg`
- `src/line_follower_interfaces/msg/GcsSnapshot.msg`
- `src/line_follower_interfaces/srv/BagCommand.srv`
- `src/line_follower_bringup/config/record_topics.json`
- `src/line_follower_bringup/launch/dev_sim.launch.py`
- `src/line_follower_mission/include/line_follower_mission/command_policy.hpp`
- `src/line_follower_mission/include/line_follower_mission/freshness.hpp`
- `src/line_follower_mission/include/line_follower_mission/snapshot.hpp`
- `src/line_follower_mission/src/mission_manager_node.cpp`
- `src/line_follower_mission/test/test_command_policy.cpp`
- `src/line_follower_mission/test/test_snapshot.cpp`
- `src/line_follower_tests/CMakeLists.txt`
- `src/line_follower_tests/test/test_checkpoint_metrics.py`
- `src/line_follower_tests/test/test_replay_policy.py`
- `src/line_follower_tests/test/test_gcs_session.py`

44 source/documentation files are listed above.

## Limits and remaining work

There are no known unimplemented required D-13/review-follow-up items. Independent lead verification
of the remediation remains. The strict requested outcome is still reported FAILED because of the
earlier non-zero acceptance runs; this report does not conceal them behind green reruns.

The rover algorithms remain dummies. Real recording showed healthy baseline perception without
fallback, and straight observed/planned geometry; fallback/unhealthy extraction, memory/candidate
rendering and quality/NaN cases are covered by synthetic/unit tests. Smartphone video synchronization
was optional and is skipped. Long-duration size/time guards are implemented but were not driven to
their 300 s/1 GiB limits; the short check independently verifies <128 MiB. Stage-2 UI validation was
offscreen using live/recorded real data, without repeating a manual host-display session. No screenshot
feature or PNG deliverable was added. No training, new autonomous driving algorithm or full-model
performance claim is made by these checks.
