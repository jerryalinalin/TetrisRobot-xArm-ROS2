#!/usr/bin/env python3
"""
Least-squares fitting example.
"""

import numpy as np

A = np.mat(np.array([[331.9, 189.9, 1],
                      [507.3, 189.9, 1],
                      [504.4, 423.3, 1],
                      [269.7, 423.3, 1]]))
B = np.mat(np.array([[75.3],
                      [77.8],
                      [81.3],
                      [75.6]]))

print(np.linalg.inv(A.transpose() * A) * A.transpose() * B)
