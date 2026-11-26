#!/usr/bin/env python3
"""Loop-play videos side by side in windows (q / Esc closes)."""
import sys
import cv2
caps = [(p, cv2.VideoCapture(p)) for p in sys.argv[1:]]
while True:
    for p, c in caps:
        ok, f = c.read()
        if not ok:
            c.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, f = c.read()
        if ok:
            cv2.imshow(p.split('/')[-1], f)
    if cv2.waitKey(100) in (27, ord('q')):
        break
