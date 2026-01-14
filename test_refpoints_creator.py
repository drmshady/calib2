"""Test refpoints_U.json creation from structure_L.json"""
import json
from datetime import datetime

# Load structure_L.json
with open('runs/phase6/structure_L.json', 'r') as f:
    structure = json.load(f)

if 'points_3d' not in structure:
    raise ValueError("structure_L.json missing 'points_3d' field")

# Convert to refpoints format
refpoints = {
    "frame": "U",
    "units": "mm",
    "points": {}
}

for track_id, point_data in structure['points_3d'].items():
    # structure_L.json uses 'xyz' field for 3D coordinates
    if 'xyz' in point_data:
        refpoints["points"][track_id] = point_data['xyz']
    elif 'position_mm' in point_data:
        refpoints["points"][track_id] = point_data['position_mm']
    else:
        raise ValueError(f"Point {track_id} missing coordinate data ('xyz' or 'position_mm')")

# Add metadata
refpoints["metadata"] = {
    "timestamp": datetime.now().isoformat() + "Z",
    "source": "structure_L.json conversion",
    "n_points": len(refpoints["points"])
}

# Test output
print(f"✓ Successfully converted {len(refpoints['points'])} points")
print(f"\nSample points:")
for i, (tid, coords) in enumerate(list(refpoints['points'].items())[:3]):
    print(f"  Track {tid}: [{coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f}] mm")
    if i >= 2:
        break

print(f"\nMetadata:")
print(f"  Timestamp: {refpoints['metadata']['timestamp']}")
print(f"  Source: {refpoints['metadata']['source']}")
print(f"  Point count: {refpoints['metadata']['n_points']}")

# Save test output
output_file = 'runs/phase6/refpoints_U_test.json'
with open(output_file, 'w') as f:
    json.dump(refpoints, f, indent=2)

print(f"\n✓ Saved to: {output_file}")
