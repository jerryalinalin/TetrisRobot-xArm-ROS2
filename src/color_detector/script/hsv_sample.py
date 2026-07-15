#!/usr/bin/env python3
"""
HSV value sampler — click on image to see HSV values.
Usage: python3 hsv_sample.py [image_path]
  image_path: path to source image (default: src.png next to script)
"""

import os
import sys
import cv2 as cv
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def drawxxx(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        print(hsv[y][x])


if __name__ == '__main__':
    src_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_SCRIPT_DIR, 'src.png')
    src = cv.imread(src_path)
    if src is None:
        print(f"Error: cannot read {src_path}", file=sys.stderr)
        sys.exit(1)

    cv.imshow("src", src)
    hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
    cv.setMouseCallback("src", drawxxx)
    cv.waitKey(0)
