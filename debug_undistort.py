#!/usr/bin/env python3
"""Debug undistortion to verify coordinate transformations."""

import numpy as np
import cv2
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src"))

from calibration_loader import load_calibration

# Load calibration
K, D, image_size, _ = load_calibration("calib/1_10/camera_intrinsics.json")

# Test point
pt_distorted = np.array([[3000.0, 2000.0]], dtype=np.float32)

# Method 1: P=K (what we're using)
pt_undist_K = cv2.undistortPoints(pt_distorted.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)

# Method 2: P=None (normalized)
pt_norm = cv2.undistortPoints(pt_distorted.reshape(-1, 1, 2), K, D, P=None).reshape(-1, 2)

# Method 3: Manual conversion from normalized
pt_manual = (K[:2, :2] @ pt_norm.T + K[:2, 2:3]).T

print(f"Original distorted point: {pt_distorted[0]}")
print(f"\nMethod 1 (P=K):     {pt_undist_K[0]}")
print(f"Method 2 (P=None):  {pt_norm[0]} (normalized)")
print(f"Method 3 (manual):  {pt_manual[0]}")

print(f"\nDifference (Method1 - Method3): {pt_undist_K[0] - pt_manual[0]}")

# Now test 3D projection
R = np.eye(3)
t = np.array([0, 0, 1000], dtype=np.float64)  # 1000mm away

X_3d = np.array([0, 0, 1000], dtype=np.float64)

# Project with K
X_proj = K @ (R @ X_3d + t)
pt_proj = X_proj[:2] / X_proj[2]

print(f"\n3D point: {X_3d}")
print(f"Projected (K @ X/Z): {pt_proj}")
print(f"Should be at image center: cx={K[0,2]:.1f}, cy={K[1,2]:.1f}")
