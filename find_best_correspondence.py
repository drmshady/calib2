#!/usr/bin/env python3
"""
Find the best marker ID correspondence by testing all permutations.
"""

import json
import numpy as np
from itertools import permutations
import sys
sys.path.insert(0, 'src')
from transforms import compute_alignment

# Load data
with open('runs/test2_multitag_full/implants_U.json') as f:
    u_data = json.load(f)

# IOS scan body coordinates (from CSV files)
ios_coords = {
    '1': [6.56891346, 9.95790005, 1.61009002],      # SB1
    '2': [-4.33664131, 0.79839635, 1.24360752],     # SB2
    '3': [-16.35935974, 4.30547619, -0.39979088],   # SB3
    '4': [-19.75767136, 18.32509995, 0.45158446]    # SB4
}

# U-frame implants (photogrammetry)
u_implants = {}
for imp_id, imp in u_data['implants'].items():
    marker_id = imp['marker_id']
    u_implants[marker_id] = np.array(imp['centroid_mm'])

print("Photogrammetry (U-frame) implants:")
for marker_id, pos in u_implants.items():
    print(f"  Marker {marker_id}: {pos}")

print("\nIOS scan body positions:")
for sb_num, pos in ios_coords.items():
    print(f"  SB{sb_num}: {pos}")

print("\n" + "="*70)
print("Testing all possible marker ID assignments...")
print("="*70)

# U-frame marker IDs
u_markers = [100, 101, 102, 103]

# Test all permutations
results = []
for perm in permutations(u_markers):
    # Create correspondence: SB1→perm[0], SB2→perm[1], SB3→perm[2], SB4→perm[3]
    
    # Build matched point sets
    centroids_u = []
    centroids_i = []
    
    for sb_num, marker_id in zip(['1', '2', '3', '4'], perm):
        centroids_u.append(u_implants[marker_id])
        centroids_i.append(np.array(ios_coords[sb_num]))
    
    centroids_u = np.array(centroids_u)
    centroids_i = np.array(centroids_i)
    
    # Compute alignment
    try:
        src_dict = {str(i): centroids_u[i] for i in range(len(centroids_u))}
        dst_dict = {str(i): centroids_i[i] for i in range(len(centroids_i))}
        
        transform, rmse, residuals = compute_alignment(
            src_dict, dst_dict,
            estimate_scale=False,
            rmse_fail_threshold=1000.0
        )
        
        results.append({
            'mapping': dict(zip(['SB1', 'SB2', 'SB3', 'SB4'], perm)),
            'rmse': rmse,
            'residuals': residuals
        })
    except Exception as e:
        pass

# Sort by RMSE
results.sort(key=lambda x: x['rmse'])

print("\nTop 10 best correspondences:\n")
for i, result in enumerate(results[:10], 1):
    print(f"#{i} RMSE: {result['rmse']:.4f} mm")
    print(f"   Mapping: {result['mapping']}")
    res_strs = [f"{float(r):.2f}" for k, r in result['residuals'].items()]
    print(f"   Residuals: {res_strs}")
    print()

print("="*70)
print(f"BEST MATCH (RMSE: {results[0]['rmse']:.4f} mm):")
print("="*70)
best = results[0]
print("\nCorrect marker ID assignment:")
for sb, marker in best['mapping'].items():
    print(f"  {sb} → Marker {marker}")

print("\nTo use this mapping, create implants_I.json with:")
for sb_num, marker_id in best['mapping'].items():
    sb_idx = sb_num.replace('SB', '')
    print(f"  SB{sb_idx} (marker {marker_id}): {ios_coords[sb_idx]}")
