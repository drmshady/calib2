#!/usr/bin/env python3
"""
Analyze pairwise distances between implants to identify possible marker ID mismatches.
If two implants have similar relative geometry, their distance matrices should match.
"""

import json
import numpy as np
from itertools import combinations

def compute_distance_matrix(implants_data):
    """Compute pairwise distances between all implants."""
    marker_ids = []
    positions = []
    
    for implant in implants_data['implants'].values():
        marker_ids.append(implant['marker_id'])
        positions.append(np.array(implant['centroid_mm']))
    
    n = len(positions)
    dist_matrix = np.zeros((n, n))
    
    for i in range(n):
        for j in range(n):
            dist_matrix[i, j] = np.linalg.norm(positions[i] - positions[j])
    
    return marker_ids, dist_matrix

# Load both datasets
with open('runs/test2_multitag_full/implants_U.json', 'r') as f:
    u_data = json.load(f)

with open('test/fixtures/real_ios_data/implants_I_recentered.json', 'r') as f:
    i_data = json.load(f)

# Compute distance matrices
u_ids, u_dist = compute_distance_matrix(u_data)
i_ids, i_dist = compute_distance_matrix(i_data)

print("="*70)
print("U-FRAME (PHOTOGRAMMETRY) - Pairwise Distances (mm)")
print("="*70)
print(f"{'':>10}", end='')
for id in u_ids:
    print(f"{id:>10}", end='')
print()
for i, id1 in enumerate(u_ids):
    print(f"{id1:>10}", end='')
    for j, id2 in enumerate(u_ids):
        print(f"{u_dist[i,j]:>10.2f}", end='')
    print()

print("\n" + "="*70)
print("I-FRAME (IOS RECENTERED) - Pairwise Distances (mm)")
print("="*70)
print(f"{'':>10}", end='')
for id in i_ids:
    print(f"{id:>10}", end='')
print()
for i, id1 in enumerate(i_ids):
    print(f"{id1:>10}", end='')
    for j, id2 in enumerate(i_ids):
        print(f"{i_dist[i,j]:>10.2f}", end='')
    print()

# Find best matching based on distance patterns
print("\n" + "="*70)
print("DISTANCE COMPARISON (should match if IDs are correct)")
print("="*70)

# Compare key distances
print("\nKey pairwise distances:")
for i in range(len(u_ids)):
    for j in range(i+1, len(u_ids)):
        u_pair_dist = u_dist[i, j]
        i_pair_dist = i_dist[i, j]
        diff = abs(u_pair_dist - i_pair_dist)
        match_str = "✓ MATCH" if diff < 5.0 else "✗ MISMATCH"
        print(f"  Marker {u_ids[i]}-{u_ids[j]}: U={u_pair_dist:6.2f}mm  I={i_pair_dist:6.2f}mm  Δ={diff:6.2f}mm  {match_str}")

print("\n" + "="*70)
print("ANALYSIS")
print("="*70)
print("If distance mismatches are large, marker IDs may be swapped.")
print("Look for permutations that minimize distance differences.")
