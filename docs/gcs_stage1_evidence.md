# Development GCS stage 1 evidence (D-11 / D-12)

Date: 2026-11-17. Workspace `/home/sion/Line_Tracer /line_follower_ws`.
All ROS and Gazebo runs used `line_follower_lab`; no host ROS, other-project container
signals, or changes to original world/model assets were used.

## Scope correction

Coordinator message `msg_5884ac38ee2a` conveyed the user's correction: **no screenshot/capture
feature and no PNG deliverable**. No screenshot parameters or flags were implemented and no
`docs/gcs_ui_live.png` or offscreen PNG was produced. The lead can capture the real host-X window.
The offscreen widget test renders and asserts its contents without saving an image.
D-13 recorder/replay remains the next task; both Bag commands currently return timeout events
when the future recorder server is absent.

## Exact one-line commands and results

Run from the quoted workspace path:

```bash
tools/lab_env.sh build
tools/lab_env.sh test
tools/lab_env.sh check-gcs
tools/lab_env.sh gcs
tools/lab_env.sh gcs-stop
tools/lab_env.sh checkpoint-a
```

All final commands exited **0**. Build: `Summary: 8 packages finished [35.3s]`.
Tests: `Summary: 68 tests, 0 errors, 0 failures, 0 skipped`.
The aggregate Checkpoint A rerun also passed all D-01 through D-04 checks.
Individual logs are `log_lab/gcs-build.log`, `gcs-test.log`, `gcs-check.log`,
`gcs-visible-final.log`, and `gcs-checkpoint-a.log`.

For manual START, use `tools/lab_env.sh gcs auto_start:=false`; the 20 s sensor gate stays in effect.
For the compatible one-shot CLI, source the lab environment and run:

```bash
ros2 run line_follower_gcs gcs_command --ros-args -p command:=kill
```

An additional no-rover invocation in domain 81 exited 0 and reported `kill: timeout` with a fresh
UUID and `service unavailable or response deadline exceeded`, without an exception
(`log_lab/gcs-cli-no-rover.log`). Active START/KILL service dispatch was tested through the same
broker in the system and node-only launch checks below.

## D-11 / D-12 headless output

`tools/lab_env.sh check-gcs` uses a real `dev_sim.launch.py headless:=true gcs:=true auto_start:=false`
launch, isolated ROS domain 82, and a unique Gazebo partition. The six pure/offscreen tests ran first.

```text
......                                                                   [100%]
6 passed in 0.07s
PASS | START before READY rejected    | rejected: mission not READY
PASS | wrong run rejected             | run_id mismatch
PASS | bag START missing recorder     | timeout: service unavailable or response deadline exceeded
PASS | bag STOP missing recorder      | timeout: service unavailable or response deadline exceeded
PASS | GCS snapshot rate              | 6 samples; 2.000 Hz
PASS | snapshot relay state equality  | 6 matched stamps; max relay=0.0006s
PASS | odom geometry and provenance   | graph=1 path=10 sources=[0]
PASS | link loss visible and isolated | LOST after 2.029s; all five rover nodes alive; mission publishes
PASS | link recovers                  | OK
PASS | START READY manual accepted    | start requested
PASS | duplicate returns original     | accepted=True; cached state=2; same response=True
PASS | START RUNNING rejected         | rejected: mission not READY
PASS | live RUNNING reaches UI feed   | GCS state=3
PASS | RUNNING display within period  | rover→GCS state delay=0.3009s (2 Hz + 50ms transport margin)
PASS | real rover moves               | odom speed=0.1000m/s
PASS | KILL active accepted           | accepted: kill requested (software stop, not a physical E-STOP)
PASS | KILL E_STOP then STOPPED       | states=[0, 1, 2, 3, 7, 8]
PASS | KILL final velocity zero       | cmd=0.0; odom=0.000000
PASS | KILL inactive rejected         | rejected: mission already inactive
PASS | timeline retains short states  | GCS transition events=[1, 2, 3, 7, 8]
PASS | live Qt map/camera render      | last render=('38', '93', '8')
PASS | live UI reception              | snapshots/camera/elapsed/Hz=('36', '88', '20.00', '1.80', '4.40')
PASS | GCS processes healthy          | []
PASS | owned process group cleaned    | live PIDs=[]
Launch log: /tmp/checkpoint-gcs-tg8b7w4z/launch.log
GCS D-11/D-12: PASS (24/24)

```

The sustained snapshot sample measured 2.000 Hz; UI average 1.80 Hz includes the deliberate
snapshot interruption, and camera average 4.40 Hz includes sensor/render startup. The live Qt
window rendered RUNNING with camera frames and subsequently rendered STOPPED. The offscreen
window test asserted 1600×1000 size, exact logical grid positions `(0,0,3,3)`, `(0,3,2,1)`,
`(2,3,1,1)`, `(3,0,1,4)`, all four map-layer updates, camera update, command button routing,
and the disabled replay control.

C++ unit tests additionally cover START rejected with auto_start enabled, pending START rejection,
KILL overriding pending START, invalid/empty requests, bounded-cache eviction and cached rejections,
and original-response preservation without re-execution. Transform tests preserve point sources
and capture stamps while rotating/translating a local path into odom. The node-only GCS launch test
uses fake sensors and domain 83; the original pipeline test verifies no competition snapshot publisher.

## Real host-X window and stop lifecycle

`tools/lab_env.sh gcs` was launched on the host's actual `DISPLAY=:0`, without offscreen mode,
and kept running until a separate `tools/lab_env.sh gcs-stop`. The final smoke waited for the first
RUNNING render before stopping. `xwininfo -display :0 -root -tree` reported:

```text
0x1800006 "Line Follower · Development Ground Station": ()  1600x1000+14+49  +116+119
```

Key final visible-run telemetry:

```text
[gcs_mission_manager-11] [INFO] [1794893209.225375365] [gcs_mission_manager]: UI_RECEIVE snapshots=9 camera=14 elapsed=5.00s snapshot_hz=1.80 camera_hz=2.80
[gcs_mission_manager-11] [INFO] [1794893214.225398442] [gcs_mission_manager]: UI_RECEIVE snapshots=19 camera=39 elapsed=10.00s snapshot_hz=1.90 camera_hz=3.90
[gcs_mission_manager-11] [INFO] [1794893219.225391784] [gcs_mission_manager]: UI_RECEIVE snapshots=29 camera=65 elapsed=15.00s snapshot_hz=1.93 camera_hz=4.33
[mission_manager-4] [INFO] [1794893223.731329090] [mission_manager]: mission state 0 -> 1 (startup delay elapsed, checking sensors)
[mission_manager-4] [INFO] [1794893223.831337539] [mission_manager]: mission state 1 -> 2 (sensors ready)
[mission_manager-4] [INFO] [1794893223.931335897] [mission_manager]: mission state 2 -> 3 (auto start)
[gcs_mission_manager-11] [INFO] [1794893224.225391731] [gcs_mission_manager]: UI_RECEIVE snapshots=39 camera=91 elapsed=20.00s snapshot_hz=1.95 camera_hz=4.55
[gcs_mission_manager-11] [INFO] [1794893224.233438678] [gcs_mission_manager]: UI_RENDER snapshots=40 camera=91 state=3
[gcs_mission_manager-11] [INFO] [1794893224.733297558] [gcs_mission_manager]: UI_RENDER snapshots=41 camera=94 state=3
[gcs_mission_manager-11] [INFO] [1794893224.811712557] [gcs_mission_manager]: UI_RECEIVE snapshots=41 camera=94 elapsed=20.59s snapshot_hz=1.99 camera_hz=4.57
[INFO] [gcs_command-10]: process has finished cleanly [pid 20034]
[INFO] [gcs_bridge-9]: process has finished cleanly [pid 20033]
[INFO] [gcs_mission_manager-11]: process has finished cleanly [pid 20035]
```

All three GCS nodes exited cleanly, with no GCS traceback or failed GCS exit in the final visible
smoke. Gazebo's upstream launch wrapper needed its normal 5 s SIGINT→SIGTERM fallback in that run;
the managed session then cleaned its owned group. The stop implementation validates the launch
PID's start-time identity, never uses pattern kills, and also backs `tools/lab_env.sh stop`.
Closing the GCS window triggers launch shutdown; Ctrl-C and SIGTERM are handled by the session.
Window-close/Ctrl-C were not separately injected; the explicit managed stop was exercised.

## Checkpoint A regression

```text
Summary: 8 packages finished [9.49s]
Summary: 68 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A test: PASS
sensors: PASS (10/10 checks)
CHECKPOINT A check-sensors: PASS
motion: PASS (14/14 checks)
CHECKPOINT A check-motion: PASS
start-stop: PASS (8/8 checks)
CHECKPOINT A check-start-stop: PASS
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
```

The rerun verified asset hashes, 20 s startup gate and zero commands before RUNNING, primitive
motion and stops, and controller-loss safety stop on the default competition launch.

## Competition dependency closure and cleanup

The exact inspection command was:

```bash
docker exec line_follower_lab bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; colcon list --packages-up-to line_follower_bringup --names-only'
```

Output (also in `log_lab/gcs-competition-closure.log`):

```text
line_follower_bringup
line_follower_control
line_follower_interfaces
line_follower_mission
line_follower_navigation
line_follower_perception

```

Neither GCS nor Qt enters that package closure; `line_follower_bringup/package.xml`, the official
competition launch and the Dockerfile were not changed. Snapshot extras are absent by default;
only `dev_sim gcs:=true` enables them. No provided assets/manifest or design §17 boxes were changed.
A final `docker exec line_follower_lab ps -eo pid,pgid,args` showed:

```text
    PID    PGID COMMAND
      1       1 /sbin/docker-init -- bash -c mkdir -p "$HOME" && exec sleep infinity
     57      57 sleep infinity
   6324    6320 /usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() --name ros2-daemon --ros-domain-id 77 --rmw-implementation rmw_fastrtps_cpp
   6926    6909 /usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() --name ros2-daemon --ros-domain-id 0 --rmw-implementation rmw_fastrtps_cpp
  21191   21191 ps -eo pid,pgid,args

```

The two ROS CLI daemons were already present before this task. No Gazebo, launch, bridge,
application or GCS process remains, and no other project's containers were stopped.

## TDD record and changed files

Expected development red states were observed before implementation: the pure model and window
modules were missing, the C++ command/snapshot headers were missing, and the new check-gcs script
was missing. An adapter test then exposed planned lines incorrectly joining across UNKNOWN points;
it failed before the implementation was corrected to split those lines. These are development
red states, not failed final acceptance checks. An intermediate visible stop exposed duplicate
SIGINT tracebacks in the Python nodes; idempotent signal handling and the identity-checked managed
stop corrected that, and the final visible run had clean GCS exits. All final acceptance checks passed.

Changed/added source and documentation (generated build/install/log files excluded):

- `README.md`
- `docs/test_plan.md`
- `docs/gcs_stage1_evidence.md`
- `src/line_follower_interfaces/CMakeLists.txt`
- `src/line_follower_interfaces/msg/GcsSnapshot.msg`
- `src/line_follower_interfaces/msg/GcsLinkStatus.msg`
- `src/line_follower_interfaces/msg/GcsCommandRequest.msg`
- `src/line_follower_interfaces/msg/GcsCommandEvent.msg`
- `src/line_follower_interfaces/srv/BagCommand.srv`
- `src/line_follower_interfaces/srv/MissionCommand.srv`
- `src/line_follower_mission/CMakeLists.txt`
- `src/line_follower_mission/include/line_follower_mission/command_policy.hpp`
- `src/line_follower_mission/include/line_follower_mission/snapshot.hpp`
- `src/line_follower_mission/src/mission_manager_node.cpp`
- `src/line_follower_mission/test/test_command_policy.cpp`
- `src/line_follower_mission/test/test_snapshot.cpp`
- `src/line_follower_bringup/config/competition.yaml`
- `src/line_follower_bringup/launch/nodes.launch.py`
- `src/line_follower_bringup/launch/dev_sim.launch.py`
- `src/line_follower_tests/CMakeLists.txt`
- `src/line_follower_tests/package.xml`
- `src/line_follower_tests/test/test_pipeline_launch.py`
- `src/line_follower_tests/test/test_gcs_launch.py`
- `tools/lab_env.sh`
- `tools/gcs_session.py`
- `tools/check_gcs.py`
- `tools/gcs/setup.py`
- `tools/gcs/package.xml`
- `tools/gcs/line_follower_gcs/gcs_bridge.py`
- `tools/gcs/line_follower_gcs/gcs_command.py`
- `tools/gcs/line_follower_gcs/gcs_mission_manager.py`
- `tools/gcs/line_follower_gcs/model.py`
- `tools/gcs/line_follower_gcs/window.py`
- `tools/gcs/line_follower_gcs/adapter.py`
- `tools/gcs/test/test_model.py`
- `tools/gcs/test/test_window.py`
- `tools/gcs/test/test_adapter.py`
- `tools/gcs/line_follower_gcs/fonts/NotoSansCJK-UI.otf`
- `tools/gcs/line_follower_gcs/fonts/COPYRIGHT`

No outstanding D-11/D-12 blockers remain. The rover's map/planning/perception algorithms are still
dummies, so the real run naturally showed straight observed/planned geometry with SOURCE_OBSERVED;
synthetic/offscreen tests exercised memory, candidate and invalid-layer rendering as well. Sustained
Wi-Fi/network impairment, a real recorder server, and replay/seek are outside this stage; D-13 will
supply the recorder and replay functions. Bag command absence was explicitly verified as timeout.
