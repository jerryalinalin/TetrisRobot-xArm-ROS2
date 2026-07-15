#!/usr/bin/env python3
"""
HSV-based color classification.
Usage: python3 classification.py [image_path]
  image_path: path to source image (default: ../src/src.png relative to script)
"""

import os
import sys
from cv2 import cv2 as cv
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(_SCRIPT_DIR, '..', 'src')


def drawxxx(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        print(hsv[y][x])


if __name__ == '__main__':
    kernel1 = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    kernel2 = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))

    src_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_SRC_DIR, 'src.png')
    src = cv.imread(src_path)
    if src is None:
        print(f"Error: cannot read {src_path}", file=sys.stderr)
        sys.exit(1)

    cv.imshow("src", src)
    hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)

    brown = cv.inRange(hsv, np.array([120, 30, 65]), np.array([170, 59, 170]))
    brown = cv.erode(brown, kernel1)
    brown = cv.dilate(brown, kernel1)
    brown = cv.dilate(brown, kernel1)
    brown = cv.erode(brown, kernel1)
    brown = cv.dilate(brown, kernel1)
    brown = cv.erode(brown, kernel1)
    cv.imshow("img", brown)

    red1 = cv.inRange(hsv, np.array([0, 120, 120]), np.array([18, 255, 190]))
    red2 = cv.inRange(hsv, np.array([160, 120, 120]), np.array([180, 255, 190]))
    red = cv.bitwise_or(red1, red2)
    red = cv.dilate(red, kernel1)
    red = cv.erode(red, kernel1)
    red = cv.dilate(red, kernel1)
    red = cv.erode(red, kernel1)

    orange1 = cv.inRange(hsv, np.array([0, 100, 190]), np.array([18, 255, 255]))
    orange2 = cv.inRange(hsv, np.array([170, 80, 190]), np.array([180, 255, 255]))
    orange = cv.bitwise_or(orange1, orange2)
    orange = cv.dilate(orange, kernel1)
    orange = cv.erode(orange, kernel1)
    orange = cv.dilate(orange, kernel1)
    orange = cv.erode(orange, kernel1)

    blue = cv.inRange(hsv, np.array([92, 160, 120]), np.array([105, 255, 255]))
    blue = cv.erode(blue, kernel2)

    green = cv.inRange(hsv, np.array([60, 100, 120]), np.array([88, 255, 255]))
    green = cv.erode(green, kernel2)
    green = cv.dilate(green, kernel2)

    purple = cv.inRange(hsv, np.array([107, 60, 60]), np.array([130, 200, 150]))
    purple = cv.erode(purple, kernel2)
    purple = cv.dilate(purple, kernel2)

    yellow = cv.inRange(hsv, np.array([20, 80, 150]), np.array([30, 255, 255]))
    yellow = cv.erode(yellow, kernel2)
    yellow = cv.dilate(yellow, kernel2)

    cv.waitKey(0)
