# Isaac Drive

Isaac Sim renders the Waffle Pi camera on the host. A TCP relay inside the
running lab container publishes ROS 2 messages and receives commands from the
existing follower. All DDS traffic for the harness stays in the container.
The harness uses the line-only USD converter and the SDF camera mount,
intrinsics, USD optical transform, RTX render step and RGB annotator from the
d-isaac capture recipe.

## Run on the Isaac host

The existing `line_follower_lab` container must be running with this checkout
mounted at `/ws`, ROS Jazzy, and the follower's Python dependencies. The host
needs Isaac Sim at `/home/rvlab/isaacsim`, a working RTX runtime, and access
to the `docker` group through `sg`. Use the bridge gateway as seen from the
container. The launcher defaults to `172.17.0.1` for both connection and bind.
Set `ISAAC_HOST` and `ISAAC_BIND` if the gateway differs. Keep the TCP port
restricted to the lab bridge because the wire protocol trusts its single peer.

First smoke the official track with the wrapper that skips Gazebo truth logging.

```bash
cd /home/rvlab/26_robotprogramming_project
ISAAC_HOME=/home/rvlab/isaacsim LAB_NAME=line_follower_lab ROS_DOMAIN_ID=109 \
  bash experiments/isaac/run_isaac_drive.sh --run-user --map official \
  --out /tmp/isaac-drive-official-smoke --port 18990 --max-sim-seconds 10 \
  --video-every 5
```

Then run a full trial on a generated map.

```bash
ISAAC_HOME=/home/rvlab/isaacsim LAB_NAME=line_follower_lab ROS_DOMAIN_ID=109 \
  bash experiments/isaac/run_isaac_drive.sh --run-user \
  --map experiments/worlds/suites/decoy1000/part0/maps/decoy_0000.json \
  --out /tmp/isaac-drive-decoy-0000 --port 18990 --max-sim-seconds 300
```

Use a fresh output path each time. Both wrappers use
`/ws/src/line_follower/config/rule.yaml`, `use_sim_time:=true`, and a 0.85 m
BEV range. Omit `--run-user` to launch the requested `run_xfar.py --xfar 0.85`
wrapper. Its Gazebo truth logger has no data in this harness, so expect no
`map_world.npz`. The startup wrapper isolates ROS discovery to localhost and
takes the bench lock for the selected container domain (90–109).

The authored material and d-isaac Key/Fill defaults (1800/350) are used
by default. Add `--lights --seed 123` for a fixed seeded lighting variation
across the trial. Add `--texture --seed 123` to apply the seeded d-isaac mat
texture. The latter uses `mat_texture.py` from the d-isaac capture checkout.
Set `ISAAC_CAPTURE_DIR` to the directory containing that helper if it has
moved. `--odom-position-std` and `--odom-yaw-std` add independent Gaussian
measurement jitter in metres per axis and radians. Both default to zero.
`--video-every N` records one RGB sample every N ticks in `camera.mp4` at
30/N fps and requires OpenCV with an MP4 encoder in Isaac's Python.

## Loop and outputs

At tick zero, the renderer sends the spawn pose and image with zero twist.
Each later tick applies the previous reply for 1/30 simulated second, with
signed limits of 0.26 m/s and 1.82 rad/s. The ramp height and nose-up
slope are sampled at the robot centre and along its heading. The official
track uses its SDF spawn, nominal hill profile and `experiments/gt.npy`.
Generated tracks use JSON spawn, ramps and ordered centre line.

The relay gives each topic the same tick timestamp and publishes `/clock`
first, then `/imu`, `/odom`, TF and `/camera/image_raw`. It spins for one
wall-time tick between publication and reply so the controller has time to
run. The reply contains the latest Twist. DDS delivery and control scheduling
are asynchronous, so a slow callback can cause a command to be reused. The
renderer still advances by exactly one simulated tick per exchange.

TCP sends a big-endian fixed header with a version marker, sequence, sim
nanoseconds, odometry and IMU fields, followed by 921,600 packed RGB bytes.
Each reply echoes the sequence and contains linear and angular velocity.
The endpoints check sequence and reject malformed or non-finite control
data. The connection has a 30-second I/O timeout. On termination the host
closes the connection, and the launcher stops the lab processes on relay
exit or closure of its control pipe.

Scoring uses truth from the kinematic pose and projects onto segments near
the highest accepted progress, as in the bench. It advances progress on
in-line gains above 3 mm, finishes within 0.12 m of the route end in arc
length, and stops after more than 0.6 simulated seconds above 0.12 m lateral
error or more than 20 simulated seconds without progress. The default
cap is 300 simulated seconds. The Gazebo scripts use configurable off-line
thresholds. This harness fixes it at the requested 0.12 m.

`result.json` is written after the other run artifacts and contains the
outcome, progress, simulated lap time, elapsed simulated and wall time,
lateral error summary and run configuration. A normal scoring abort
produces a result with `complete: false`. The launcher exits nonzero for
runtime errors. `trajectory.npy` has columns `[sim_s, x_m, y_m, candidate_s_m,
lateral_error_m]`. `isaac.log` and `lab.log` contain startup and failure
diagnostics.

## Limits and verification

This is a pose-driven camera rig with ideal unicycle motion and no visible
robot body, wheel slip, contact dynamics, collisions or acceleration limits.
Odometry is planar in heading with ground-following elevation. IMU orientation
tracks the synthetic slope, and acceleration includes approximate tangential
and lateral terms and +g projected into the body frame. Sensor rates are
30 Hz throughout. Collision, wheel-contact and slip metrics require physics.
Wall-clock speed depends on RTX and ROS callback time.

CPU test command from the repo root

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s experiments/isaac/tests -v
bash -n experiments/isaac/run_isaac_drive.sh
```

The tests exercise actual map metadata, kinematics, ramp pitch, scoring,
packet fragmentation/error handling and a fake-renderer/fake-relay loop
using the production protocol. The loopback TCP variant skips when socket
creation or connection is restricted. Real RTX rendering (including texture
and MP4), the Docker bridge, ROS messaging and follower performance need
the Isaac-host smoke test.
