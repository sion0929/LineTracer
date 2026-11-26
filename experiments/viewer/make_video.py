#!/usr/bin/env python3
"""Compose a bench run's video frames: Gazebo camera (left) + line map being built (right).

  python3 experiments/viewer/make_video.py JOB_DIR OUT.mp4 "title" [fps]
"""
import sys
from pathlib import Path
import cv2
import numpy as np

job, out, title = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
fps = float(sys.argv[4]) if len(sys.argv) > 4 else 10
cams = sorted((job / "video").glob("cam_*.jpg"))
W, H = 640 + 480 + 24, 480 + 70
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
font = cv2.FONT_HERSHEY_SIMPLEX
for k, c in enumerate(cams):
    m = job / "video" / c.name.replace("cam_", "map_").replace(".jpg", ".png")
    if not m.exists():
        continue
    canvas = np.full((H, W, 3), 20, np.uint8)
    canvas[50:530, 8:648] = cv2.imread(str(c))
    canvas[50:530, 656:1136] = cv2.imread(str(m))
    cv2.putText(canvas, title, (10, 30), font, 0.75, (240, 240, 240), 2, cv2.LINE_AA)
    cv2.putText(canvas, f"t = {k * 0.2:5.1f} s", (W - 150, 30), font, 0.65, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Gazebo camera 640x480", (14, 70), font, 0.5, (0, 230, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, "line map (odom): white line, green path, orange = BEV", (662, 70), font, 0.45, (0, 230, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, "lines: yellow = BEV near / split, blue = far end; blue band = 640 far band", (10, 545), font, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
    vw.write(canvas)
vw.release()
print(out, len(cams), "frames")
