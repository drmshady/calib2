"""Debug: Check if observations are undistorted."""

import sys
sys.path.insert(0, 'd:/calib2/src')

import json
import numpy as np

# Load structure
with open('calib/test/structure_L.json') as f:
    data = json.load(f)

# Get one point with observations
point_id = '4'  # First point
point = data['points_3d'][point_id]
xyz = np.array(point['xyz'])

# Get first camera
cam_id = 'DSC_0276'
cam = data['cameras'][cam_id]
K = np.array(cam['K'])
R = np.array(cam['R'])
t = np.array(cam['t']).reshape(3, 1)

# Get observation
obs = np.array(point['observations'][cam_id])

print(f"Point {point_id} @ {xyz}")
print(f"Camera {cam_id}")
print(f"Observed 2D: {obs}")

# Project
X_cam = R @ xyz + t.ravel()
X_proj = K @ X_cam
pt_proj = X_proj[:2] / X_proj[2]

print(f"Projected 2D: {pt_proj}")
print(f"Reprojection error: {np.linalg.norm(obs - pt_proj):.3f}px")

# Check if observation looks like distorted or undistorted
# Undistorted should be close to principal point for points near optical axis
print(f"\nPrincipal point (cx, cy): ({K[0,2]:.1f}, {K[1,2]:.1f})")
print(f"Observation distance from principal point: {np.linalg.norm(obs - [K[0,2], K[1,2]]):.1f}px")
