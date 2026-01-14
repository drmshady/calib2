#!/usr/bin/env python3
"""
Convert IOS scan body center coordinates (CSV) to implants_I.json format.

Usage:
    python tools/convert_ios_centers.py --input centers.csv --output implants_I.json --marker-ids 100 101 102 103
"""

import argparse
import json
import numpy as np
from pathlib import Path


def load_centers_csv(csv_path):
    """
    Load scan body centers from CSV file (space or comma separated).
    
    Supports two formats:
    1. Position only (3 columns): x y z
    2. With tag ID (4 columns): tag_id x y z  OR  x y z tag_id
    
    Returns:
        List of tuples: [(tag_id, [x, y, z]), ...] or [(None, [x, y, z]), ...]
    """
    centers = []
    with open(csv_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('//'):
                continue
            
            # Try space-separated first, then comma-separated
            parts = line.replace(',', ' ').split()
            
            if len(parts) == 3:
                # Format: x y z (no tag ID)
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                centers.append((None, [x, y, z]))
            elif len(parts) == 4:
                # Try to detect which column is tag ID
                # Tag ID is typically an integer (100-103), coordinates are floats
                try:
                    # Try first column as tag ID
                    tag_id = int(parts[0])
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    centers.append((tag_id, [x, y, z]))
                except ValueError:
                    # Try last column as tag ID
                    try:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        tag_id = int(parts[3])
                        centers.append((tag_id, [x, y, z]))
                    except ValueError:
                        print(f"Warning: Line {line_num} has 4 columns but couldn't parse tag ID, skipping")
                        continue
            else:
                print(f"Warning: Line {line_num} has {len(parts)} columns (expected 3 or 4), skipping")
                continue
    
    return centers


def create_implants_json(centers_with_ids, marker_ids, output_path, axis_direction='z'):
    """
    Create implants_I.json from center coordinates.
    
    Args:
        centers_with_ids: List of tuples [(tag_id, [x,y,z]), ...]
        marker_ids: List of marker IDs (only used if CSV doesn't have tag IDs)
        output_path: Output JSON file path
        axis_direction: Default axis direction ('x', 'y', 'z', or '-x', '-y', '-z')
    """
    # Check if CSV has tag IDs
    has_tag_ids = any(tag_id is not None for tag_id, _ in centers_with_ids)
    
    if has_tag_ids:
        # Use tag IDs from CSV
        print("Using tag IDs from CSV file")
        centers_dict = {}
        for tag_id, coords in centers_with_ids:
            if tag_id is not None:
                centers_dict[tag_id] = coords
        
        if len(centers_dict) != len(centers_with_ids):
            print(f"Warning: Some rows missing tag IDs, using {len(centers_dict)} rows with tag IDs")
    else:
        # Use provided marker IDs
        centers = [coords for _, coords in centers_with_ids]
        if len(centers) != len(marker_ids):
            raise ValueError(f"Number of centers ({len(centers)}) must match number of marker IDs ({len(marker_ids)})")
        
        centers_dict = {marker_ids[i]: centers[i] for i in range(len(centers))}
    
    # Parse axis direction
    axis_map = {
        'x': [1, 0, 0], '-x': [-1, 0, 0],
        'y': [0, 1, 0], '-y': [0, -1, 0],
        'z': [0, 0, 1], '-z': [0, 0, -1]
    }
    
    if axis_direction not in axis_map:
        raise ValueError(f"Invalid axis direction: {axis_direction}. Must be one of {list(axis_map.keys())}")
    
    default_axis = axis_map[axis_direction]
    
    # Build implants data
    implants_data = {
        "frame": "I",
        "units": "mm",
        "implants": {}
    }
    
    for idx, (marker_id, center) in enumerate(sorted(centers_dict.items()), start=1):
        implants_data["implants"][str(idx)] = {
            "centroid_mm": center,
            "axis_vector": default_axis,
            "marker_id": int(marker_id),
            "scanbody_name": f"SB{idx:02d}",
            "fit_rms_mm": None
        }
    
    # Save JSON
    with open(output_path, 'w') as f:
        json.dump(implants_data, f, indent=2)
    
    print(f"Converted {len(centers_dict)} scan body centers to implants format")
    print(f"Output: {output_path}")
    print("\nImplants:")
    for idx, (marker_id, center) in enumerate(sorted(centers_dict.items()), start=1):
        print(f"  SB{idx:02d} (marker {marker_id}): {center}")
    print(f"\nDefault axis direction: {axis_direction} = {default_axis}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert IOS scan body centers (CSV) to implants_I.json format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic conversion (marker IDs on command line)
    python tools/convert_ios_centers.py --input centers.csv --output implants_I.json --marker-ids 100 101 102 103
    
    # CSV with tag ID column (marker IDs in file)
    python tools/convert_ios_centers.py --input centers_with_ids.csv --output implants_I.json
    
    # Specify axis direction
    python tools/convert_ios_centers.py --input centers.csv --output implants_I.json --marker-ids 100 101 102 103 --axis z
    
    # Different axis (if IOS uses different convention)
    python tools/convert_ios_centers.py --input centers.csv --output implants_I.json --marker-ids 100 101 102 103 --axis -y

CSV Format:
    Option 1 - Position only (3 columns, requires --marker-ids):
        -19.757 18.325 0.451
        -4.336 0.798 1.243
        6.568 9.957 1.610
        -16.359 4.305 -0.399
    
    Option 2 - With tag ID (4 columns, tag_id x y z):
        100 -19.757 18.325 0.451
        101 -4.336 0.798 1.243
        102 6.568 9.957 1.610
        103 -16.359 4.305 -0.399
    
    Option 3 - Tag ID at end (x y z tag_id):
        -19.757 18.325 0.451 100
        -4.336 0.798 1.243 101
        6.568 9.957 1.610 102
        -16.359 4.305 -0.399 103
        """
    )
    
    parser.add_argument('--input', '-i', required=True,
                        help='Input CSV file with scan body centers')
    parser.add_argument('--output', '-o', required=True,
                        help='Output implants_I.json file')
    parser.add_argument('--marker-ids', nargs='+', type=int, required=False,
                        help='AprilTag marker IDs for each scan body (e.g., 100 101 102 103). Required if CSV does not have tag ID column.')
    parser.add_argument('--axis', default='z',
                        choices=['x', 'y', 'z', '-x', '-y', '-z'],
                        help='Default axis direction for scan bodies (default: z)')
    
    args = parser.parse_args()
    
    # Validate inputs
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}")
        return 1
    
    # Load centers
    try:
        centers_with_ids = load_centers_csv(input_path)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return 1
    
    if len(centers_with_ids) == 0:
        print("Error: No valid center coordinates found in CSV file")
        return 1
    
    # Check if CSV has tag IDs
    has_tag_ids = any(tag_id is not None for tag_id, _ in centers_with_ids)
    
    if not has_tag_ids and not args.marker_ids:
        print("Error: CSV file does not contain tag IDs and --marker-ids not provided")
        print("\nEither:")
        print("  1. Add tag ID column to CSV (format: tag_id x y z)")
        print("  2. Provide --marker-ids argument (e.g., --marker-ids 100 101 102 103)")
        return 1
    
    if not has_tag_ids and len(centers_with_ids) != len(args.marker_ids):
        print(f"Error: Number of centers ({len(centers_with_ids)}) must match number of marker IDs ({len(args.marker_ids)})")
        return 1
    
    # Convert
    try:
        create_implants_json(centers_with_ids, args.marker_ids or [], args.output, args.axis)
    except Exception as e:
        print(f"Error creating implants JSON: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
