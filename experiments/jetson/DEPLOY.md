# Deploying the ROS 2 Jazzy line follower on a Jetson Orin Nano 8GB

This is the concrete path from a freshly flashed Orin Nano to a running
`ros2 launch line_follower ...` inside the container, plus the latency/power
measurements.  The host stays Ubuntu 22.04 (JetPack 6.x); ROS 2 Jazzy lives in
the container because Jazzy targets Ubuntu 24.04.

Two values are site-specific.  Set them once in the shell used on the
development PC and on the Orin:

```bash
ORIN_IP=192.168.1.50
ORIN_USER=orin
```

Everything else below is literal.

---

## 1. Base image decision

`experiments/jetson/Dockerfile.jazzy-l4t` uses:

```
dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04
sha256:ad157785a7aeea59506c0ddd6c8c28d3d75c4bdf64a5da804f5f00ff6c9154ad
```

Why this exact image:

* It is an **L4T r36.4.0 / JetPack 6.1** userspace image.  Orin Nano is supported
  by JetPack 6.x.
* Its container userspace is **Ubuntu 24.04**, the release ROS 2 Jazzy is built
  for.  The host can remain Ubuntu 22.04; only the container is 24.04.
* It already contains **ROS 2 Jazzy ros_base**, **CUDA 12.8**, **cuDNN 9.7**,
  **TensorRT 10.7** and an aarch64 **OpenCV 4.11** build.  This is the
  combination that actually works on the board.
* NVIDIA's `nvcr.io/nvidia/l4t-jetpack:r36.4.0` base is Ubuntu 22.04, where
  `ros-jazzy-*` apt packages do not exist.  Docker Hub's `ros:jazzy` image has no
  L4T GPU userspace.  Neither is a drop-in.
* It is pinned by digest in the Dockerfile, so a rebuild is reproducible.

Check the board before building:

```bash
cat /etc/nv_tegra_release
# R36 (release), REVISION: 4.0 ...  -> matches the image
```

If the board reports r36.3.x (JetPack 6.0), reflash to JetPack 6.1 or newer, or
rebuild with a matching `dustynv/ros:jazzy-*-r36.3.*` tag.  If it reports
r36.4.3/r36.4.4 (JetPack 6.2), the r36.4.0 userspace is normally compatible;
verify with the provider check in step 5.  Do not mix a container CUDA minor
version with an older host driver: that is the usual
`CUDA driver version is insufficient` failure.

`ros_gz_bridge` and `ros_gz_sim` are **not installed**.  The robot camera
publishes ROS images directly; there is no Gazebo on the robot.

### onnxruntime-gpu index

The build ARG is:

```
JETSON_AI_LAB_INDEX=https://pypi.jetson-ai-lab.io/jp6/cu128
```

It is passed to pip as an extra index.  The Jazzy 24.04 base is Python 3.12; the
current JP6/cu128 index publishes cp310 wheels only.  pip therefore also
consults PyPI for the aarch64 cp312 `onnxruntime-gpu` wheel, and the Dockerfile
**fails the build** unless the installed package reports
`TensorrtExecutionProvider` or `CUDAExecutionProvider`.  If you specifically
need NVIDIA's Jetson-built cp310 wheel, use a Python 3.10 L4T base (Ubuntu 22.04)
and build ROS 2 Jazzy from source; that is a different image.  For the usual
Jazzy image, the checked cp312 GPU wheel is the working choice.

---

## 2. Prepare the Orin

### 2.1 Docker and the NVIDIA runtime

On a fresh JetPack 6 the Docker engine is usually already present.  Install the
NVIDIA container runtime and register it with Docker:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo usermod -aG docker "$USER"      # log out and back in once
docker info | grep -i runtime
```

`docker info` must list `nvidia`.  If the package is missing, enable the NVIDIA
JetPack apt repository first (`sudo apt-get update` after adding it), then repeat
the commands above.

### 2.2 Copy the repository to the board

From the development PC:

```bash
scp -r 26_robotprogramming_project "${ORIN_USER}@${ORIN_IP}:~/"
ssh "${ORIN_USER}@${ORIN_IP}"
cd ~/26_robotprogramming_project
```

The image is built **on the Orin** (aarch64).  Do not cross-build it on an x86
PC.

---

## 3. Build the image

From the repository root on the Orin:

```bash
cd ~/26_robotprogramming_project
docker build -f experiments/jetson/Dockerfile.jazzy-l4t \
  -t line_follower:jazzy-l4t .
```

The first build downloads the ~5.5 GB L4T Jazzy base plus CUDA/TensorRT.  The
Dockerfile installs `python3-opencv`, `numpy`, `ros-jazzy-ros-base` (already in
the base), the Jetson AI Lab extra index, `onnxruntime-gpu`, `onnx`, and the
`to_trt.sh` / `run_bench_with_tegrastats.sh` helpers.  To use a different base
or index:

```bash
docker build -f experiments/jetson/Dockerfile.jazzy-l4t \
  --build-arg L4T_BASE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg JETSON_AI_LAB_INDEX=https://pypi.jetson-ai-lab.io/jp6/cu128 \
  -t line_follower:jazzy-l4t .
```

---

## 4. Verify the image

```bash
docker run --rm --runtime nvidia line_follower:jazzy-l4t \
  python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"

docker run --rm line_follower:jazzy-l4t \
  bash -lc 'ros2 pkg prefix line_follower && ros2 pkg executables line_follower'
```

Expected: the provider list contains `TensorrtExecutionProvider` and/or
`CUDAExecutionProvider`, and `follower_node` is listed.  If only
`CPUExecutionProvider` appears, the container was started without
`--runtime nvidia` or the wheel has no GPU provider; see Troubleshooting.

---

## 5. Power mode and camera

Put the board in the highest power mode for measurements:

```bash
sudo nvpmodel -m 0          # MAXN
sudo jetson_clocks
sudo nvpmodel -q
```

Start the camera publisher.  The robot uses a MIPI CSI-2 camera and the camera
node must publish `sensor_msgs/Image` on `/camera/image_raw`.  Use the same
camera node the lab uses; start it on the host (the container shares the host
network) or inside the container from a second shell.  Do not start two
publishers on the same topic.

For a CSI-2 camera the device nodes and Argus socket are passed into the
container in step 6 (`/dev/video0`, `/dev/video1`, `/tmp/argus_socket`).  Verify
from the host that the devices exist:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

If you use a UVC/USB camera instead and have a ROS 2 Jazzy V4L2 camera node, run
it with the topic remapped to `/camera/image_raw`:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640,480]" \
  -r image_raw:=/camera/image_raw
```

Then verify the topic:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --field width --once
```

`/odom` and `/imu` must also be published by the base driver, and `/cmd_vel`
must be consumed by it.  The follower node uses exactly these four topics.

---

## 6. Run the follower in the container

Create the results directory on the host so the container can write into it:

```bash
mkdir -p experiments/results/lead/ds/w2 experiments/jetson/trt_cache
```

Start an interactive container:

```bash
cd ~/26_robotprogramming_project
docker run --rm -it --name line_follower \
  --runtime nvidia \
  --network host \
  --ipc host \
  --device /dev/video0 \
  --group-add video \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /usr/bin/tegrastats:/host/usr/bin/tegrastats:ro \
  -v "$PWD/experiments/jetson/trt_cache:/ws/experiments/jetson/trt_cache" \
  -v "$PWD/experiments/results/lead/ds/w2:/ws/experiments/results/lead/ds/w2" \
  line_follower:jazzy-l4t
```

Add `--device /dev/video1` to the command above if the CSI driver exposes a
second video node.  Inside the container:

```bash
ros2 launch line_follower robot.launch.py
```

This is the robot launch file (`robot.launch.py`), not the Gazebo launch
(`line_follower.launch.py`).  It starts `follower_node` with
`use_sim_time:=false`, `perception:=rule`, and the learned line restorer
`/ws/experiments/jetson/models/V1-Restore128.onnx`.

Useful launch arguments:

```bash
ros2 launch line_follower robot.launch.py camera_topic:=/camera/image_raw
ros2 launch line_follower robot.launch.py restore_model:=/ws/experiments/jetson/models/V3-Restore128.onnx
ros2 launch line_follower robot.launch.py restore_model:=""
ros2 launch line_follower robot.launch.py cmd_vel_topic:=/cmd_vel
```

`perception:=dl` is a stub in this repository; keep `perception:=rule` unless a
segmentation model is wired into `line_follower/perception/dl.py`.

In a second terminal on the host, verify the command stream:

```bash
docker exec -it line_follower /ws/entrypoint.sh bash -lc 'ros2 topic hz /cmd_vel'
```

Keep the robot on blocks for the first run.  The `speed_profile` and `v_max`
defaults are tuned for the simulation track; tune them for the real track with
`ros2 param set /follower_node v_max 0.2` or a YAML file.

---

## 7. Measure latency and resources

### 7.1 Pure TensorRT engines with trtexec

This builds an FP16 engine for every `experiments/jetson/models/*.onnx` with
fixed shapes `1x2x128x128` (restore) and `1x3x240x320` (segmentation), and saves
timing/profile JSON plus a text summary:

```bash
docker run --rm --runtime nvidia \
  -v "$PWD/experiments/results/lead/ds/w2:/ws/experiments/results/lead/ds/w2" \
  -e OUT_DIR=/ws/experiments/results/lead/ds/w2/engines \
  -e LOG_DIR=/ws/experiments/results/lead/ds/w2/trt_logs \
  line_follower:jazzy-l4t \
  /ws/experiments/jetson/to_trt.sh
```

Results:

* `experiments/results/lead/ds/w2/engines/*_fp16_<shape>.engine`
* `experiments/results/lead/ds/w2/trt_logs/*_fp16.log`
* `.../*_fp16_times.json`, `.../*_fp16_profile.json`
* `.../trt_logs/timings.txt` — one summary block per model

`trtexec` uses `--warmUp=500 --duration=10 --avgRuns=100`; the log contains
`GPU Compute Time`, `Latency` (median and percentile 99) and `Throughput`.

### 7.2 The actual node backends with bench_models.py + tegrastats

This runs every ONNX through the onnxruntime providers and OpenCV DNN that the
node can use, while sampling `tegrastats` every 200 ms:

```bash
docker run --rm --runtime nvidia \
  -e TEGRASTATS=/host/usr/bin/tegrastats \
  -v /usr/bin/tegrastats:/host/usr/bin/tegrastats:ro \
  -v "$PWD/experiments/results/lead/ds/w2:/ws/experiments/results/lead/ds/w2" \
  -v "$PWD/experiments/jetson/trt_cache:/ws/experiments/jetson/trt_cache" \
  line_follower:jazzy-l4t \
  /ws/experiments/jetson/run_bench_with_tegrastats.sh --runs 200 --threads 4
```

(If `tegrastats` is already inside the image, drop the bind mount and the
`TEGRASTATS` variable.  If the mounted binary cannot load its libraries, run
`sudo tegrastats --interval 200 --logfile /tmp/tegrastats.log` on the host and
run `bench_models.py` in the container separately; the raw log is still
summarisable.)

Outputs in `experiments/results/lead/ds/w2/`:

* `bench_<host>.json` — median / p95 / max per model and backend
* `tegrastats_<host>_<stamp>.log` — raw 200 ms samples
* `tegrastats_<host>_<stamp>_summary.json` and `.txt` — GPU %, CPU %, RAM,
  CPU/GPU/SOC/tj temperature, VDD_IN power and energy
* `env_<host>_<stamp>.txt` — `nvpmodel -q`, `jetson_clocks --show`, kernel and
  `/etc/nv_tegra_release`

The summary printed at the end is the table to paste into the report.  Read it
together with the README budget: the segmentation model runs every frame, the
restoration model only when the traced line runs out, and the frame budget is
33 ms at 30 Hz.  Compare `median_ms` and `p95_ms` against the TensorRT targets
in `experiments/jetson/README.md`.

### 7.3 End-to-end run

While the follower is driving, run `tegrastats` in a second terminal and keep
the log:

```bash
sudo tegrastats --interval 200 --logfile /tmp/line_follower_run_tegrastats.log
```

Record the mission duration, `/camera/image_raw` rate, `/cmd_vel` rate and the
raw tegrastats log.  For a comparable power number, use the same `nvpmodel`
mode (`MAXN`) and `jetson_clocks` state as the bench run.

---

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `could not select device driver "" with capabilities: [[gpu]]` | Install `nvidia-container-toolkit`, run `sudo nvidia-ctk runtime configure --runtime=docker`, restart Docker, and always pass `--runtime nvidia`. |
| `CUDA driver version is insufficient` or `libcudart.so.12` missing | Host L4T and container L4T do not match.  Check `cat /etc/nv_tegra_release`; use a matching `dustynv/ros:jazzy-*-r36.x.y-*` base or reflash the host. |
| `onnxruntime` lists only `CPUExecutionProvider` | Start with `--runtime nvidia`; check `python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"` inside the container.  If the wheel has no GPU provider, rebuild with `--build-arg ONNXRUNTIME_GPU_SPEC=...` from the Jetson AI Lab index / a matching base. |
| `trtexec: command not found` | The TensorRT tarball is at `/usr/src/tensorrt/bin/trtexec`; the Dockerfile symlinks it to `/usr/local/bin/trtexec`.  Set `TRTEXEC=/usr/src/tensorrt/bin/trtexec` if needed. |
| `tegrastats` not found inside the container | `tegrastats` is part of the host L4T BSP.  Mount `/usr/bin/tegrastats` at `/host/usr/bin/tegrastats` and set `TEGRASTATS=/host/usr/bin/tegrastats`. |
| Camera topic is empty | Check `/dev/video*`, pass the devices, add `--group-add video`, mount `/tmp/argus_socket`, and use `--network host` with the same `ROS_DOMAIN_ID` on host and container.  Do not run two publishers on `/camera/image_raw`. |
| `/cmd_vel` is published but the robot does not move | The base driver must subscribe to `/cmd_vel` and publish `/odom` and `/imu`.  Check `ros2 topic info /cmd_vel` and `ros2 topic hz /odom`. |
| Node waits forever after launch | A simulation YAML with `use_sim_time: true` was loaded.  On the robot use `robot.launch.py` (it sets `use_sim_time:=false`) or pass `--ros-args -p use_sim_time:=false`. |
| `restore_model not found` | Use the image path `/ws/experiments/jetson/models/V1-Restore128.onnx`, or pass `restore_model:=/ws/experiments/jetson/models/V3-Restore128.onnx`.  The node falls back to no restoration when the file is absent. |
| Docker build is extremely slow on an x86 PC | Build on the Orin.  The base is aarch64 L4T; emulation is not a supported deployment path. |
