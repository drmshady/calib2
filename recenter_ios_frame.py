#!/usr/bin/env python3
"""
Recenter IOS I-frame to place origin at SB01 (marker 100) instead of SB02.
This aligns the I-frame coordinate system with the photogrammetry U-frame.
"""

import json
import numpy as np

# Load original IOS data
with open('test/fixtures/real_ios_data/implants_I.json', 'r') as f:
    ios_data = json.load(f)

# Get SB01 (marker 100) position - this will become the new origin
sb01_pos = np.array(ios_data['implants']['1']['centroid_mm'])
print(f"Original SB01 position (will become origin): {sb01_pos}")
print(f"\nShifting all implants by: {-sb01_pos}")

# Transform all implant positions
for implant_id, implant in ios_data['implants'].items():
    original_pos = np.array(implant['centroid_mm'])
    new_pos = original_pos - sb01_pos
    implant['centroid_mm'] = new_pos.tolist()
    
    print(f"\nMarker {implant['marker_id']} ({implant['scanbody_name']}):")
    print(f"  Original: {original_pos}")
    print(f"  Recentered: {new_pos}")

# Save recentered data
output_path = 'test/fixtures/real_ios_data/implants_I_recentered.json'
with open(output_path, 'w') as f:
    json.dump(ios_data, f, indent=2)

print(f"\n✓ Saved recentered I-frame to: {output_path}")
print("\nNow SB01 (marker 100) is at origin, matching photogrammetry U-frame")
