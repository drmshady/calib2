#!/usr/bin/env python3
"""Debug script to check triangulation issue."""

import sys
from pathlib import Path
from glob import glob
import numpy as np
import cv2
import json

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from tools.phase3_test_pipeline import detect_apriltags, load_calibration, undistort_points, get_2d_points_from_detections

# Test parameters
image_paths = sorted(glob("calib/test/DSC_*.TIF"))[:2]
calib_file = "calib/1_10/camera_intrinsics.json"
tag_size_mm = 7.0

# Load calibration
K, D, image_size, _ = load_calibration(calib_file)
print(f"K = \n{K}")
print(f"D = {D}")

# Detect in first two images
det1 = detect_apriltags(image_paths[0])
det2 = detect_apriltags(image_paths[1])

print(f"\nImage 1: {image_paths[0]}")
print(f"  Tags detected: {len(det1)}")
for d in det1:
    print(f"    Tag {d['tag_id']}: corners shape = {d['corners'].shape}")
    print(f"      Corner 0: {d['corners'][0]}")

print(f"\nImage 2: {image_paths[1]}")
print(f"  Tags detected: {len(det2)}")
for d in det2:
    print(f"    Tag {d['tag_id']}: corners shape = {d['corners'].shape}")

# Get 2D points
points_2d_1 = get_2d_points_from_detections(det1)
points_2d_2 = get_2d_points_from_detections(det2)

print(f"\n2D points extraction:")
print(f"  Image 1: {points_2d_1.shape} = {points_2d_1[:2]}")
print(f"  Image 2: {points_2d_2.shape} = {points_2d_2[:2]}")

# Undistort
points_2d_1_undist = undistort_points(points_2d_1, K, D, output="pixels")
points_2d_2_undist = undistort_points(points_2d_2, K, D, output="pixels")

print(f"\nUndistorted points:")
print(f"  Image 1: {points_2d_1_undist.shape} = {points_2d_1_undist[:2]}")
print(f"  Image 2: {points_2d_2_undist.shape} = {points_2d_2_undist[:2]}")

# PnP for pose estimation
half_size = tag_size_mm / 2.0
tag_corners_3d = np.array([
    [-half_size, -half_size, 0],
    [ half_size, -half_size, 0],
    [ half_size,  half_size, 0],
    [-half_size,  half_size, 0],
], dtype=np.float32)

print(f"\nTag 3D template (tag size = {tag_size_mm}mm):")
print(tag_corners_3d)

# PnP for view 1 (first tag)
print(f"\nSolving PnP for view 1...")
success, rvec1, tvec1 = cv2.solvePnP(
    tag_corners_3d,
    det1[0]["corners"],
    K, D,
    flags=cv2.SOLVEPNP_ITERATIVE
)
print(f"  Success: {success}")
print(f"  rvec: {rvec1.ravel()}")
print(f"  tvec: {tvec1.ravel()}")

R1, _ = cv2.Rodrigues(rvec1)
print(f"  R:\n{R1}")

# PnP for view 2 (first tag)
print(f"\nSolving PnP for view 2...")
success, rvec2, tvec2 = cv2.solvePnP(
    tag_corners_3d,
    det2[0]["corners"],
    K, D,
    flags=cv2.SOLVEPNP_ITERATIVE
)
print(f"  Success: {success}")
print(f"  rvec: {rvec2.ravel()}")
print(f"  tvec: {tvec2.ravel()}")

R2, _ = cv2.Rodrigues(rvec2)

# Relative pose
R_rel = R2 @ R1.T
t_rel = (tvec2 - R2 @ R1.T @ tvec1).reshape(3, 1)

print(f"\nRelative pose:")
print(f"  R_rel:\n{R_rel}")
print(f"  t_rel: {t_rel.ravel()}")
print(f"  Baseline: {np.linalg.norm(tvec2 - tvec1):.2f}mm")

# Projection matrices
P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
P2 = K @ np.hstack([R_rel, t_rel])

print(f"\nProjection matrices:")
print(f"  P1 =\n{P1}")
print(f"  P2 =\n{P2}")

# Try triangulating first point
print(f"\nTriangulating first point:")
pt1 = points_2d_1_undist[0]
pt2 = points_2d_2_undist[0]
print(f"  pt1: {pt1}")
print(f"  pt2: {pt2}")

A = np.array([
    pt1[0] * P1[2] - P1[0],
    pt1[1] * P1[2] - P1[1],
    pt2[0] * P2[2] - P2[0],
    pt2[1] * P2[2] - P2[1]
])

print(f"  A matrix:\n{A}")

_, _, Vt = np.linalg.svd(A)
X_hom = Vt[-1]
X_cam1 = X_hom[:3] / X_hom[3]

print(f"  X_hom: {X_hom}")
print(f"  X_cam1: {X_cam1}")
print(f"  depth1: {X_cam1[2]:.2f}mm")

# Check depth in camera 2
X_cam2 = R_rel @ X_cam1.reshape(3, 1) + t_rel
depth2 = X_cam2[2, 0]
print(f"  depth2: {depth2:.2f}mm")

# Reprojection errors
pt1_proj = K @ X_cam1
pt1_reproj = pt1_proj[:2] / pt1_proj[2]
error1 = np.linalg.norm(pt1 - pt1_reproj)
print(f"  Reprojection error view 1: {error1:.2f}px")

pt2_proj = K @ X_cam2.ravel()
pt2_reproj = pt2_proj[:2] / pt2_proj[2]
error2 = np.linalg.norm(pt2 - pt2_reproj)
print(f"  Reprojection error view 2: {error2:.2f}px")

print(f"\n  Valid? depth1>0: {depth1>0}, depth2>0: {depth2>0}, err1<5: {error1<5}, err2<5: {error2<5}")
