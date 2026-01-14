#!/usr/bin/env python3
"""
Test for scale mismatch and coordinate system transformations.
"""

import json
import numpy as np
import sys
sys.path.insert(0, 'src')
from transforms import compute_alignment

# Load data
with open('runs/test2_multitag_full/implants_U.json') as f:
    u_data = json.load(f)

# IOS coordinates - best correspondence from previous test
ios_mapping = {
    101: [6.56891346, 9.95790005, 1.61009002],      # SB1
    103: [-4.33664131, 0.79839635, 1.24360752],     # SB2
    102: [-16.35935974, 4.30547619, -0.39979088],   # SB3
    100: [-19.75767136, 18.32509995, 0.45158446]    # SB4
}

# U-frame implants
u_implants = {}
for imp in u_data['implants'].values():
    u_implants[imp['marker_id']] = np.array(imp['centroid_mm'])

# Build matched point sets
markers = [100, 101, 102, 103]
centroids_u = np.array([u_implants[m] for m in markers])
centroids_i = np.array([ios_mapping[m] for m in markers])

print("="*70)
print("TEST 1: Rigid alignment (no scale)")
print("="*70)

src_dict = {str(i): centroids_u[i] for i in range(len(centroids_u))}
dst_dict = {str(i): centroids_i[i] for i in range(len(centroids_i))}

T_rigid, rmse_rigid, _ = compute_alignment(src_dict, dst_dict, estimate_scale=False, rmse_fail_threshold=1000.0)
print(f"RMSE (rigid): {rmse_rigid:.4f} mm\n")

print("="*70)
print("TEST 2: Similarity transform (with scale estimation)")
print("="*70)

T_sim, rmse_sim, residuals_sim = compute_alignment(src_dict, dst_dict, estimate_scale=True, rmse_fail_threshold=1000.0)
print(f"RMSE (with scale): {rmse_sim:.4f} mm")
print(f"Scale factor: {T_sim.s:.6f}")
print(f"Improvement: {(rmse_rigid - rmse_sim):.4f} mm ({(rmse_rigid - rmse_sim)/rmse_rigid*100:.1f}%)\n")

print("="*70)
print("TEST 3: Manual scale factors")
print("="*70)

# Test various scale factors
scale_factors = [0.1, 0.2, 0.25, 0.3, 0.33, 0.4, 0.5, 2.0, 3.0, 4.0, 5.0]

best_scale = None
best_rmse = float('inf')

for scale in scale_factors:
    centroids_i_scaled = centroids_i * scale
    dst_dict_scaled = {str(i): centroids_i_scaled[i] for i in range(len(centroids_i_scaled))}
    
    try:
        T, rmse, _ = compute_alignment(src_dict, dst_dict_scaled, estimate_scale=False, rmse_fail_threshold=1000.0)
        print(f"Scale {scale:.2f}x: RMSE = {rmse:.4f} mm")
        
        if rmse < best_rmse:
            best_rmse = rmse
            best_scale = scale
    except:
        pass

print(f"\nBest manual scale: {best_scale}x with RMSE {best_rmse:.4f} mm\n")

print("="*70)
print("TEST 4: Coordinate system transformations")
print("="*70)

# Test various axis flips and swaps
transforms = [
    ("Original (X, Y, Z)", lambda p: p),
    ("Flip X (-X, Y, Z)", lambda p: np.array([-p[0], p[1], p[2]])),
    ("Flip Y (X, -Y, Z)", lambda p: np.array([p[0], -p[1], p[2]])),
    ("Flip Z (X, Y, -Z)", lambda p: np.array([p[0], p[1], -p[2]])),
    ("Swap XY (Y, X, Z)", lambda p: np.array([p[1], p[0], p[2]])),
    ("Swap XZ (Z, Y, X)", lambda p: np.array([p[2], p[1], p[0]])),
    ("Swap YZ (X, Z, Y)", lambda p: np.array([p[0], p[2], p[1]])),
    ("Flip XY (-X, -Y, Z)", lambda p: np.array([-p[0], -p[1], p[2]])),
    ("Flip XZ (-X, Y, -Z)", lambda p: np.array([-p[0], p[1], -p[2]])),
    ("Flip YZ (X, -Y, -Z)", lambda p: np.array([p[0], -p[1], -p[2]])),
]

results = []
for name, transform in transforms:
    centroids_i_transformed = np.array([transform(p) for p in centroids_i])
    dst_dict_transformed = {str(i): centroids_i_transformed[i] for i in range(len(centroids_i_transformed))}
    
    try:
        T, rmse, _ = compute_alignment(src_dict, dst_dict_transformed, estimate_scale=False, rmse_fail_threshold=1000.0)
        results.append((name, rmse))
    except:
        pass

# Sort by RMSE
results.sort(key=lambda x: x[1])

print("\nCoordinate system test results (top 5):\n")
for i, (name, rmse) in enumerate(results[:5], 1):
    print(f"{i}. {name}: RMSE = {rmse:.4f} mm")

print("\n" + "="*70)
print("TEST 5: Combined scale + coordinate transform")
print("="*70)

# Test best coordinate transform with scale estimation
best_coord_name, _ = results[0]
best_coord_idx = [name for name, _ in transforms].index(best_coord_name)
best_transform = transforms[best_coord_idx][1]

centroids_i_transformed = np.array([best_transform(p) for p in centroids_i])
dst_dict_transformed = {str(i): centroids_i_transformed[i] for i in range(len(centroids_i_transformed))}

T_combined, rmse_combined, _ = compute_alignment(src_dict, dst_dict_transformed, estimate_scale=True, rmse_fail_threshold=1000.0)

print(f"Best coordinate transform: {best_coord_name}")
print(f"With scale estimation:")
print(f"  RMSE: {rmse_combined:.4f} mm")
print(f"  Scale: {T_combined.s:.6f}x")

print("\n" + "="*70)
print("SUMMARY")
print("="*70)
print(f"Original (rigid):                 {rmse_rigid:.4f} mm")
print(f"With scale estimation:            {rmse_sim:.4f} mm (scale={T_sim.s:.4f}x)")
print(f"Best manual scale:                {best_rmse:.4f} mm (scale={best_scale}x)")
print(f"Best coordinate transform:        {results[0][1]:.4f} mm ({results[0][0]})")
print(f"Best combined (transform+scale):  {rmse_combined:.4f} mm")

if rmse_combined < 5.0:
    print(f"\n✓ SUCCESS: RMSE < 5.0mm achieved!")
    print(f"\nRecommended fix:")
    print(f"  1. Apply coordinate transform: {best_coord_name}")
    print(f"  2. Scale IOS data by: {T_combined.s:.6f}x")
elif rmse_combined < 10.0:
    print(f"\n⚠ MARGINAL: RMSE still {rmse_combined:.2f}mm (threshold 5.0mm)")
    print(f"Consider verifying physical setup matches between scans")
else:
    print(f"\n✗ FAIL: Even with optimal transforms, RMSE = {rmse_combined:.2f}mm")
    print(f"This indicates fundamentally different implant configurations")
    print(f"Photogrammetry and IOS data are likely from different cases")
