#!/usr/bin/env python3
"""Show images in windows on the desktop (fit to 1800x1000). q / Esc closes all."""
import sys
import cv2
for p in sys.argv[1:]:
    im = cv2.imread(p)
    s = min(1.0, 1800 / im.shape[1], 1000 / im.shape[0])
    if s < 1:
        im = cv2.resize(im, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    cv2.namedWindow(p.split('/')[-1], cv2.WINDOW_AUTOSIZE)
    cv2.imshow(p.split('/')[-1], im)
while cv2.waitKey(100) not in (27, ord('q')):
    pass
