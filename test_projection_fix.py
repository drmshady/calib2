"""Test that projection formula is fixed."""

import sys
sys.path.insert(0, 'd:/calib2/src')

import numpy as np
from bundle_adjustment import project_point, matrix_to_rodrigues
from geometry_utils import compute_reprojection_error

# Setup simple camera
K = np.array([[1000, 0, 500],
              [0, 1000, 400],
              [0, 0, 1]])

R = np.eye(3)
t = np.zeros(3)
rvec = matrix_to_rodrigues(R)

# Test point at (0, 0, 100) in camera frame
# Should project to principal point (500, 400)
X = np.array([0, 0, 100])

# Test project_point
pt_proj = project_point(X, rvec, t, K)
print(f"project_point result: {pt_proj}")
print(f"Expected: [500, 400]")
print(f"Error: {np.linalg.norm(pt_proj - [500, 400]):.6f}px")

# Test compute_reprojection_error
# If we observe [500, 400], error should be ~0
error = compute_reprojection_error(
    X.reshape(1, 3),
    np.array([[500, 400]]),
    K, R, t.reshape(3, 1),
    return_per_point=True
)[0]
print(f"\ncompute_reprojection_error: {error:.6f}px")
print(f"Expected: ~0.0")

# Test off-center point
X2 = np.array([10, -5, 100])  # 10mm right, 5mm down at Z=100mm
# Should project to (500 + 1000*10/100, 400 - 1000*5/100) = (600, 350)
pt_proj2 = project_point(X2, rvec, t, K)
print(f"\nOff-center point projection: {pt_proj2}")
print(f"Expected: [600, 350]")
print(f"Error: {np.linalg.norm(pt_proj2 - [600, 350]):.6f}px")
