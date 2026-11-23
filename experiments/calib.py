#!/usr/bin/env python3
"""Static projection check: place the robot at known poses (dev-only set_pose), grab a camera frame,
project dark pixels with follower's model and report the error against the mesh centreline."""
import math, os, subprocess, sys, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import follower as F  # noqa: E402

USE_IMU = '--imu' in sys.argv
POSES = [(2.30, 0.33, 0.0), (0.33, 0.70, math.pi / 2), (2.10, 1.95, math.pi / 2)]


def teleport(x, y, yaw):
    req = f'name: "turtlebot3_waffle_pi", position: {{x: {x}, y: {y}, z: 0.01}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
    subprocess.run(['gz', 'service', '-s', '/world/line_track/set_pose', '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req], check=True, capture_output=True)


def main():
    env = dict(os.environ, ROS_DOMAIN_ID='63', GZ_PARTITION=f'cal_{os.getpid()}'); os.environ.update(env)
    sim = subprocess.Popen(['ros2', 'launch', 'line_follower_bringup', 'sim.launch.py', 'headless:=true'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True, env=env)
    seg = np.load(HERE / 'segments.npy')[:, :, :2]
    a0 = seg[:, 0]; d = seg[:, 1] - seg[:, 0]; dd = (d * d).sum(1)
    rclpy.init(); node = rclpy.create_node('calib'); frames = []; odom = []
    node.create_subscription(Image, '/camera/image_raw', lambda m: frames.append((time.monotonic(), m)), qos_profile_sensor_data)
    def on_odom(m):
        q = m.pose.pose.orientation
        odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    node.create_subscription(Odometry, '/odom', on_odom, 100)
    imu = []
    def on_imu(m):
        q = m.orientation
        imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                    math.asin(max(-1., min(1., 2 * (q.w * q.y - q.z * q.x)))), math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))))
    node.create_subscription(Imu, '/imu', on_imu, 400)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    v0, us, vs, lx, ly = F.ground_lut()
    def spin(sec, cmd=None):
        t = time.time()
        while time.time() - t < sec:
            if cmd is not None: pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
    try:
        t = time.time()
        while not frames and time.time() - t < 60:
            rclpy.spin_once(node, timeout_sec=0.2)
        for name, (x, y, yaw), (v, w) in [('drive', (1.95, 0.33, 0.0), (0.2, 0.0)), ('rotate', (2.55, 0.33, 0.0), (0.0, 1.5)), ('arc', (2.45, 0.33, 0.0), (0.2, 1.2))]:
            teleport(x, y, yaw); spin(2.0, Twist()); frames.clear(); odom.clear(); imu.clear()
            cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
            spin(1.6, cmd); spin(0.5, Twist())
            # odom frame -> world: odom pose at the first sample corresponds to the teleport pose
            o = np.array(odom); ox, oy, oa = o[0, 1], o[0, 2], o[0, 3]
            im = np.array(imu); iyaw = np.unwrap(im[:, 1]); iyaw = iyaw - iyaw[0] + oa
            if USE_IMU:
                o[:, 3] = np.interp(o[:, 0], im[:, 0], iyaw)
            def world(px, py):
                c, s = math.cos(yaw - oa), math.sin(yaw - oa)
                dx, dy = px - ox, py - oy
                return x + c * dx - s * dy, y + s * dx + c * dy
            print(name, 'frames', len(frames), 'odom', len(o))
            stamps = [(f.header.stamp.sec + f.header.stamp.nanosec * 1e-9) for _, f in frames]
            print('  image stamp - latest odom stamp at arrival (median):',
                  round(float(np.median([s_ - o[np.searchsorted(o[:, 0], s_) - 1, 0] if np.searchsorted(o[:, 0], s_) > 0 else 0 for s_ in stamps])), 4))
            print('  pitch range deg', np.degrees(im[:, 2]).min().round(2), np.degrees(im[:, 2]).max().round(2), 'roll range', np.degrees(im[:, 3]).min().round(2), np.degrees(im[:, 3]).max().round(2))
            for mode, lat in [('flat', 0.04), ('pr+', 0.04), ('pr-roll', 0.04), ('p_only', 0.04), ('pr_neg', 0.04)]:
                errs = []
                for (_, f), st in zip(frames[5:-15], stamps[5:-15]):
                    tq = st - lat
                    k = np.searchsorted(o[:, 0], tq)
                    if k <= 0 or k >= len(o): continue
                    r = (tq - o[k - 1, 0]) / (o[k, 0] - o[k - 1, 0])
                    px_, py_ = o[k - 1, 1] + r * (o[k, 1] - o[k - 1, 1]), o[k - 1, 2] + r * (o[k, 2] - o[k - 1, 2])
                    da = math.atan2(math.sin(o[k, 3] - o[k - 1, 3]), math.cos(o[k, 3] - o[k - 1, 3])); pa = o[k - 1, 3] + r * da
                    img = np.frombuffer(f.data, np.uint8).reshape(f.height, f.width, -1)
                    j = int(np.argmin(np.abs(im[:, 0] - st)))
                    p0, r0 = im[j, 2] - im[:5, 2].mean(), im[j, 3] - im[:5, 3].mean()
                    pp, rr = {'flat': (0, 0), 'pr+': (p0, r0), 'pr-roll': (p0, -r0), 'p_only': (p0, 0), 'pr_neg': (-p0, -r0)}[mode]
                    gx, gy, ok0 = F.project(pp, rr)
                    dark = (img[..., :3].mean(2)[F.RAY_V, F.RAY_U].ravel() < 80)[ok0]
                    LX, LY = gx[ok0][dark], gy[ok0][dark]
                    c, s = math.cos(pa), math.sin(pa)
                    qx = px_ + c * LX - s * LY; qy = py_ + s * LX + c * LY
                    P = np.c_[world(qx, qy)]
                    tt = np.clip(((P[:, None, :] - a0[None]) * d[None]).sum(2) / dd[None], 0, 1)
                    near = a0[None] + tt[..., None] * d[None]
                    errs.append(np.linalg.norm(near - P[:, None, :], axis=2).min(1).mean())
                print(f'  {mode:8s}: mean err {np.mean(errs):.4f}  p90 {np.percentile(errs,90):.4f}')
    finally:
        os.killpg(sim.pid, 2); sim.wait(15)


if __name__ == '__main__':
    main()
