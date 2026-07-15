#!/usr/bin/env python3
"""
K-means color clustering for block segmentation.
Usage: python3 k_means.py [work_dir]
  work_dir: directory containing src.png, output dst.png (default: script directory)
"""

import os
import sys
import cv2 as cv
import numpy as np

work_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))

src_path = os.path.join(work_dir, 'src.png')
dst_path = os.path.join(work_dir, 'dst.png')

img = cv.imread(src_path)
if img is None:
    print(f"Error: cannot read {src_path}", file=sys.stderr)
    sys.exit(1)

data = img.reshape((-1, 3))
data = np.float32(data)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 20, 0.1)
flags = cv.KMEANS_RANDOM_CENTERS

compactness, labels, centers = cv.kmeans(data, 3, None, criteria, 10, flags)

centers = np.uint8(centers)
for center in centers:
    print(center)
res = centers[labels.flatten()]
dst = res.reshape(img.shape)
cv.imwrite(dst_path, dst)
print(f"K-means done, output: {dst_path}")
