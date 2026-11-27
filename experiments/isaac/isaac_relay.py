#!/usr/bin/env python3
"""Translate Isaac Drive TCP frames to ROS 2 inside the lab container."""

import argparse
import math
import os
import socket
import time

# ISAAC_HOLD: wall-time periods to hold each sim tick (>1 gives slow followers headroom under load;
# sim time is unaffected, so results stop depending on host CPU load).
HOLD = float(os.environ.get('ISAAC_HOLD', '1'))

from drive_core import DT, HEIGHT, HZ, WIDTH, Command, orientation, receive_frame, send_command


def connect(host, port, deadline):
    while True:
        try:
            peer = socket.create_connection((host, port), timeout=2)
        except (ConnectionRefusedError, TimeoutError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.2)
        else:
            peer.settimeout(float(os.environ.get('ISAAC_RELAY_TIMEOUT', '120')))
            peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return peer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="172.17.0.1")
    parser.add_argument("--port", type=int, default=18990)
    parser.add_argument("--startup-seconds", type=float, default=900)
    parser.add_argument("--map-dump", action="store_true")
    args = parser.parse_args()
    if not 0 < args.port < 65536 or not math.isfinite(args.startup_seconds) or args.startup_seconds <= 0:
        parser.error("invalid port or startup timeout")

    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu
    from tf2_ros import TransformBroadcaster

    latest = Command(0.0, 0.0)

    def on_command(msg):
        nonlocal latest
        v, w = msg.linear.x, msg.angular.z
        if math.isfinite(v) and math.isfinite(w):
            latest = Command(v, w)

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("isaac_drive_relay")
        node.create_subscription(Twist, "/cmd_vel", on_command, 10)
        clock = node.create_publisher(Clock, "/clock", 1)
        image = node.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        odom = node.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
        truth = (node.create_publisher(Odometry, "/isaac/true_odom", qos_profile_sensor_data)
                 if args.map_dump else None)
        imu = node.create_publisher(Imu, "/imu", qos_profile_sensor_data)
        tf = TransformBroadcaster(node)

        required = (clock, image, truth if args.map_dump else odom, imu)
        deadline = time.monotonic() + args.startup_seconds
        while not (node.count_publishers("/cmd_vel") and
                   all(pub.get_subscription_count() for pub in required)):
            if time.monotonic() >= deadline:
                raise TimeoutError("Follower ROS publishers/subscribers were not discovered")
            rclpy.spin_once(node, timeout_sec=0.1)
        with connect(args.host, args.port, deadline) as peer:
            node.get_logger().info(f"Connected to Isaac at {args.host}:{args.port}")
            tick = 0
            while rclpy.ok():
                try:
                    frame = receive_frame(peer)
                except EOFError:
                    if truth is not None:
                        # Allow the terminal image and pose to drain before the supervisor stops the follower.
                        time.sleep(DT * HOLD)
                    break
                if frame.tick != tick or frame.stamp_ns != round(tick * 1_000_000_000 / HZ):
                    raise ValueError(f"Unexpected timestamp or sequence at tick {tick}")
                sec, ns = divmod(frame.stamp_ns, 1_000_000_000)
                stamp = Time(sec=sec, nanosec=ns)
                clock.publish(Clock(clock=stamp))

                inertial = Imu()
                inertial.header.stamp = stamp
                inertial.header.frame_id = "imu_link"
                q = inertial.orientation
                q.x, q.y, q.z, q.w = frame.imu_q
                omega = inertial.angular_velocity
                omega.x, omega.y, omega.z = frame.angular_velocity
                a = inertial.linear_acceleration
                a.x, a.y, a.z = frame.acceleration
                imu.publish(inertial)

                state = Odometry()
                state.header.stamp = stamp
                state.header.frame_id = "odom"
                state.child_frame_id = "base_footprint"
                p, q = state.pose.pose.position, state.pose.pose.orientation
                p.x, p.y, p.z = frame.position
                q.x, q.y, q.z, q.w = frame.odom_q
                state.twist.twist.linear.x = frame.twist.v
                state.twist.twist.angular.z = frame.twist.w
                odom.publish(state)

                if truth is not None:
                    actual = Odometry()
                    actual.header.stamp = stamp
                    actual.header.frame_id = "world"
                    actual.child_frame_id = state.child_frame_id
                    p_true, q_true = actual.pose.pose.position, actual.pose.pose.orientation
                    p_true.x, p_true.y, p_true.z, yaw = frame.truth
                    q_true.x, q_true.y, q_true.z, q_true.w = orientation(yaw, 0)
                    actual.twist = state.twist
                    truth.publish(actual)

                transform = TransformStamped()
                transform.header = state.header
                transform.child_frame_id = state.child_frame_id
                transform.transform.translation.x = p.x
                transform.transform.translation.y = p.y
                transform.transform.translation.z = p.z
                transform.transform.rotation = q
                tf.sendTransform(transform)

                camera = Image()
                camera.header.stamp = stamp
                camera.header.frame_id = "camera_rgb_frame"
                camera.height, camera.width = HEIGHT, WIDTH
                camera.encoding = "rgb8"
                camera.step = WIDTH * 3
                camera.data = frame.rgb
                image.publish(camera)

                # DDS and the follower run asynchronously. Hold sim time steady
                # for one wall-time period while collecting the latest Twist.
                until = time.monotonic() + DT * HOLD
                while (remaining := until - time.monotonic()) > 0 and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=remaining)
                send_command(peer, tick, latest)
                tick += 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
