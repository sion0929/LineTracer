#!/usr/bin/env python3
"""Show the robot camera (/camera/image_raw) 1:1 at its native 640x480 in a window, with fps."""
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = rclpy.create_node('live_view')
    st = {'img': None, 'n': 0, 't': time.monotonic(), 'fps': 0.0}

    def on_image(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        st['img'] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
        st['n'] += 1
        now = time.monotonic()
        if now - st['t'] > 1.0:
            st['fps'] = st['n'] / (now - st['t']); st['n'] = 0; st['t'] = now

    node.create_subscription(Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
    cv2.namedWindow('camera 640x480', cv2.WINDOW_AUTOSIZE)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        if st['img'] is not None:
            img = st['img'].copy()
            h, w = img.shape[:2]
            cv2.putText(img, f'{w}x{h}  {st["fps"]:.1f} fps', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(img, f'{w}x{h}  {st["fps"]:.1f} fps', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1)
            cv2.imshow('camera 640x480', img)
        if cv2.waitKey(1) in (27, ord('q')):
            break


if __name__ == '__main__':
    main()
