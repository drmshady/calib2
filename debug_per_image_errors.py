"""Check per-image reprojection errors."""

import sys
sys.path.insert(0, 'd:/calib2/src')

import json
import numpy as np

# Load structure
with open('calib/test/structure_L.json') as f:
    data = json.load(f)

# Compute per-image errors
image_errors = {}

for cam_id, cam in data['cameras'].items():
    K = np.array(cam['K'])
    R = np.array(cam['R'])
    t = np.array(cam['t']).reshape(3, 1)
    
    errors = []
    
    for point_id, point in data['points_3d'].items():
        if cam_id not in point['observations']:
            continue
        
        xyz = np.array(point['xyz'])
        obs = np.array(point['observations'][cam_id])
        
        # Project
        X_cam = R @ xyz + t.ravel()
        X_proj = K @ X_cam
        pt_proj = X_proj[:2] / X_proj[2]
        
        error = np.linalg.norm(obs - pt_proj)
        errors.append(error)
    
    if errors:
        image_errors[cam_id] = {
            'mean': np.mean(errors),
            'max': np.max(errors),
            'n_obs': len(errors)
        }

# Sort by mean error
sorted_images = sorted(image_errors.items(), key=lambda x: x[1]['mean'], reverse=True)

print("Per-Image Reprojection Errors (worst first):")
print("-" * 60)
for img_id, stats in sorted_images:
    print(f"{img_id}: mean={stats['mean']:.3f}px, max={stats['max']:.3f}px, n={stats['n_obs']}")

print("\n" + "=" * 60)
all_means = [stats['mean'] for _, stats in image_errors.items()]
print(f"Overall mean: {np.mean(all_means):.3f}px")
print(f"Overall max: {np.max([stats['max'] for _, stats in image_errors.items()]):.3f}px")
