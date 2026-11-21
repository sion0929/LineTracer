# Checkpoint A implementation evidence

Date: 2026-11-17. Workspace: `/home/sion/Line_Tracer /line_follower_ws`.
All ROS/Gazebo commands ran inside `line_follower_lab` using `tools/lab_env.sh`;
no host-side ROS commands or other-project process signals were used.

## Commands and outcomes

All final acceptance commands exited **0**:

```bash
tools/lab_env.sh build
tools/lab_env.sh test
tools/lab_env.sh check-sensors
tools/lab_env.sh check-motion
tools/lab_env.sh check-start-stop
tools/lab_env.sh checkpoint-a
```

Build: `Summary: 8 packages finished [2.05s]`.
Standalone tests: `Summary: 53 tests, 0 errors, 0 failures, 0 skipped`.
The original five-node launch test and mission/safety unit tests needed no fixes.

Standalone runtime evidence is saved in `log_lab/checkpoint_a/checkpoint-sensors.log`,
`checkpoint-motion.log`, and `checkpoint-start-stop.log`. The aggregate final run below
uses the final checker code (including reliable application-topic subscriptions and
peak-twist bounds); its complete output is `log_lab/checkpoint_a/checkpoint-a.log`.

## Final aggregate PASS/FAIL tables

```text
Summary: 8 packages finished [5.65s]
Summary: 53 tests, 0 errors, 0 failures, 0 skipped
CHECKPOINT A test: PASS
PASS | /camera/image_raw              | 151 messages/5s; ['sensor_msgs/msg/Image']
PASS | /camera/camera_info            | 151 messages/5s; ['sensor_msgs/msg/CameraInfo']
PASS | /imu                           | 1000 messages/5s; ['sensor_msgs/msg/Imu']
PASS | /odom                          | 250 messages/5s; ['nav_msgs/msg/Odometry']
PASS | /clock                         | 1251 messages/5s; ['rosgraph_msgs/msg/Clock']
PASS | camera dimensions and rate     | 640x480; measured 30.30 Hz wall (world target 30 Hz)
PASS | velocity bridge subscriber     | [('checkpoint_a_sensors', 'geometry_msgs/msg/Twist'), ('ros_gz_bridge', 'geometry_msgs/msg/Twist')]
PASS | no scoped bridge interfaces    | ROS graph hits=[]; bridge config scanned
PASS | provided asset SHA256          | 12 files verified; 
PASS | owned process group cleaned    | live PIDs=[]
sensors: PASS (10/10 checks)
CHECKPOINT A check-sensors: PASS
PASS | forward                        | dx=+0.0706m dy=+0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after forward             | stop=0.052s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | backward                       | dx=-0.0713m dy=-0.0000m yaw=+0.0000rad; peak=0.060/0.000
PASS | stop after backward            | stop=0.080s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | left turn                      | dx=+0.0439m dy=+0.0136m yaw=+0.5920rad; peak=0.040/0.500
PASS | stop after left turn           | stop=0.047s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | right turn                     | dx=+0.0456m dy=-0.0148m yaw=-0.6180rad; peak=0.040/0.500
PASS | stop after right turn          | stop=0.056s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate left                    | dx=-0.0000m dy=-0.0000m yaw=+0.7176rad; peak=0.000/0.600
PASS | stop after rotate left         | stop=0.048s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | rotate right                   | dx=-0.0000m dy=+0.0000m yaw=-0.7128rad; peak=0.000/0.600
PASS | stop after rotate right        | stop=0.060s; 2s drift=0.00000m yaw=0.00000; speed=0.00000/0.00000
PASS | bounded excursion              | max from spawn odom=0.0919m
PASS | owned process group cleaned    | live PIDs=[]
motion: PASS (14/14 checks)
CHECKPOINT A check-motion: PASS
PASS | 20 second startup gate         | elapsed=20.205s; launch wall=20.707s
PASS | all pre-RUNNING commands zero  | 856 samples; nonzero=0
PASS | pre-RUNNING odom stationary    | 855 samples; displacement=0.000000m
PASS | dummy pipeline drives          | odom speed=0.0254m/s
PASS | controller loss command stop   | PID=8686; zero after 0.209s; allowed=0.580s
PASS | controller loss physical stop  | stop after 0.256s; final speed=(1.6523241108679088e-14, 1.2995315238153782e-14)
PASS | safety explains stop           | ['', 'command candidate stale']
PASS | owned process group cleaned    | live PIDs=[]
start-stop: PASS (8/8 checks)
CHECKPOINT A check-start-stop: PASS
CHECKPOINT A: PASS (D-01 D-02 D-03 D-04)
```

## TDD and scope

Before implementation, `tools/lab_env.sh check-sensors` exited 1 because that subcommand
was absent. The new measurement tests were then run with
`python3 -m pytest -q src/line_follower_tests/test/test_checkpoint_metrics.py` and failed
collection with `FileNotFoundError: tools/checkpoint_metrics.py`; after implementing the
predicates all 12 tests passed, and they are now registered with ament/colcon.
These were expected development red states, not failed final acceptance runs.
The final aggregate command and every standalone runtime check passed without a failed
runtime acceptance run. `bash -n tools/lab_env.sh` and Python compilation also passed.

Added:

- `tools/checkpoint_a.py`: dev-only runtime sensor/motion/start-stop checker.
- `tools/checkpoint_metrics.py`: pure pose/rate/direction measurement predicates.
- `src/line_follower_tests/test/test_checkpoint_metrics.py`: 12 measurement test cases.
- `docs/checkpoint_a_evidence.md`: this report.

Changed:

- `tools/lab_env.sh`: individual checks, aggregate command, isolated test ROS domain,
  and propagation of colcon failures before result reporting.
- `src/line_follower_tests/CMakeLists.txt`: register measurement tests.
- `README.md`: lab commands and behavior.
- `docs/test_plan.md`: D/T mapping and exact acceptance tolerances.

No runtime C++ logic, launch profiles, package dependencies, original worlds/models,
`SHA256SUMS`, autonomous strategy, or design §17 checkboxes changed.
The competition graph still has only safety_guard publishing the final velocity command.
The checker uses ROS domain 79 and a unique Gazebo partition; colcon launch tests use 78.
All runtime checks clean their own process group and print `live PIDs=[]`.
A final `docker exec line_follower_lab ps -eo pid,pgid,args` showed only container init,
its idle sleep, the two ROS CLI daemons already present before this task (domains 77 and 0),
and the inspection command: no Gazebo, launch, bridge, or application processes remain.
Other `uav_ugv_*` containers were left untouched.

## Limits

The pipeline remains a dummy straight-line pipeline; this verifies Checkpoint A rather
than autonomous line following. All required normal-run sensor, drive, start-gate and
controller-interruption behavior was exercised. Ctrl-C and generic-exception zero-command
cleanup are implemented with Python-owned signal handling before ROS shutdown, but a
separate deliberate Ctrl-C/failure injection was not run. Startup zero assertions cover
all command samples received by subscriptions created before launching; DDS discovery
can precede the first received sample, and there is no excluded pre-transition guard band.
Sensor rate thresholds intentionally allow a slower CPU down to 5 Hz; motion/stop deadlines
use wall time and may fail under heavy simulation overload, as documented in the test plan.
There are no outstanding implementation blockers or required-check deviations.
