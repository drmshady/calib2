#!/usr/bin/env python3
"""
Convert IOS Scan Body Data to Implant Schema Format.

Transforms scan body poses (4×4 homogeneous matrices) from IOS/exocad
into the standardized implant schema with centroids and axis vectors.

Input Format:
    - IOS JSON with T_world_from_scanbody matrices
    - Frame: IOS_WORLD (exocad coordinate system)

Output Format:
    - implants_I.json following calib/fixtures/implant_schema.json
    - Centroids: Translation component (last column)
    - Axes: Z-axis direction (3rd column of rotation matrix)

Usage:
    python tools/convert_ios_scanbody.py \\
        --input data/ios_scanbody_poses.json \\
        --output data/implants_I.json \\
        --marker-ids 100 101 102 103

Author: Phase 6 IOS Integration Pipeline
"""

import argparse
import json
import sys
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
import numpy as np


def extract_centroid_and_axis_from_matrix(T: np.ndarray) -> tuple:
    """Extract centroid and axis from 4×4 transformation matrix.
    
    Args:
        T: 4×4 homogeneous transformation matrix T_world_from_scanbody
        
    Returns:
        Tuple of (centroid, axis):
            - centroid: (3,) position in mm
            - axis: (3,) unit vector along implant axis
    """
    T = np.array(T)
    
    # Extract translation (last column, first 3 rows)
    centroid = T[:3, 3]
    
    # Extract Z-axis direction (3rd column, first 3 rows)
    # Assuming scan body Z-axis aligns with implant longitudinal axis
    axis = T[:3, 2]
    
    # Normalize axis (should already be unit, but ensure)
    axis = axis / np.linalg.norm(axis)
    
    return centroid, axis


def is_identity_matrix(T: np.ndarray, tolerance: float = 1e-6) -> bool:
    """Check if matrix is approximately identity.
    
    Args:
        T: 4×4 transformation matrix
        tolerance: Maximum deviation from identity
        
    Returns:
        True if matrix is identity (not aligned)
    """
    T = np.array(T)
    identity = np.eye(4)
    return np.allclose(T, identity, atol=tolerance)


def convert_ios_scanbody_to_implants(
    ios_data: Dict,
    marker_ids: Optional[List[int]] = None,
    skip_identity: bool = True
) -> Dict:
    """Convert IOS scan body data to implant schema format.
    
    Args:
        ios_data: Input dictionary with IOS scan body poses
        marker_ids: List of AprilTag marker IDs to assign (e.g., [100,101,102,103])
        skip_identity: If True, skip scan bodies with identity transforms (not aligned)
        
    Returns:
        Dictionary in implant schema format
        
    Raises:
        ValueError: If data format is invalid
    """
    if "implants" not in ios_data:
        raise ValueError("Missing 'implants' field in IOS data")
    
    implants_out = {}
    skipped = []
    implant_id = 1
    
    for i, sb_data in enumerate(ios_data["implants"]):
        sb_name = sb_data.get("name", f"SB{i+1:02d}")
        
        if "T_world_from_scanbody" not in sb_data:
            print(f"Warning: {sb_name} missing transformation matrix, skipping")
            skipped.append(sb_name)
            continue
        
        T = np.array(sb_data["T_world_from_scanbody"])
        
        # Check for identity matrix (not aligned)
        if skip_identity and is_identity_matrix(T):
            print(f"Warning: {sb_name} has identity transform (not aligned), skipping")
            skipped.append(sb_name)
            continue
        
        # Extract centroid and axis
        centroid, axis = extract_centroid_and_axis_from_matrix(T)
        
        # Assign marker ID
        if marker_ids and len(marker_ids) >= implant_id:
            marker_id = marker_ids[implant_id - 1]
        else:
            marker_id = 99 + implant_id  # Default: 100, 101, 102, ...
        
        # Build implant entry
        implants_out[str(implant_id)] = {
            "centroid_mm": centroid.tolist(),
            "axis_vector": axis.tolist(),
            "marker_id": marker_id,
            "scanbody_name": sb_name,
            "fit_rms_mm": sb_data.get("fit", {}).get("rms_mm")
        }
        
        implant_id += 1
    
    # Summary
    print(f"\nConverted {len(implants_out)} implants:")
    for imp_id, imp_data in implants_out.items():
        sb_name = imp_data["scanbody_name"]
        marker_id = imp_data["marker_id"]
        centroid = imp_data["centroid_mm"]
        print(f"  Implant {imp_id}: {sb_name} → marker {marker_id} at [{centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}]")
    
    if skipped:
        print(f"\nSkipped {len(skipped)} scan bodies: {', '.join(skipped)}")
    
    return implants_out


def main():
    parser = argparse.ArgumentParser(
        description="Convert IOS scan body poses to implant schema format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Convert with default marker IDs (100-103)
    python tools/convert_ios_scanbody.py \\
        --input data/ios_scanbody_poses.json \\
        --output data/implants_I.json
    
    # Specify custom marker IDs
    python tools/convert_ios_scanbody.py \\
        --input data/ios_scanbody_poses.json \\
        --output data/implants_I.json \\
        --marker-ids 100 101 102 103
    
    # Include identity transforms (not recommended)
    python tools/convert_ios_scanbody.py \\
        --input data/ios_scanbody_poses.json \\
        --output data/implants_I.json \\
        --include-identity
        """
    )
    
    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="Path to IOS scan body JSON file"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Path to save implants_I.json"
    )
    
    parser.add_argument(
        "--marker-ids",
        type=int,
        nargs="+",
        help="AprilTag marker IDs to assign (e.g., 100 101 102 103)"
    )
    
    parser.add_argument(
        "--include-identity",
        action="store_true",
        help="Include scan bodies with identity transforms (not aligned)"
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Convert IOS Scan Body Data to Implant Schema")
    print("=" * 70)
    print()
    
    # Load IOS data
    print(f"Loading IOS data from: {args.input}")
    with open(args.input, 'r') as f:
        ios_data = json.load(f)
    
    print(f"  Case ID: {ios_data.get('case_id', 'Unknown')}")
    print(f"  Frame: {ios_data.get('frame', 'Unknown')}")
    print(f"  Total scan bodies: {len(ios_data.get('implants', []))}")
    
    # Convert
    print(f"\nConverting scan body poses...")
    implants = convert_ios_scanbody_to_implants(
        ios_data,
        marker_ids=args.marker_ids,
        skip_identity=not args.include_identity
    )
    
    # Build output
    output_data = {
        "frame": "I",
        "units": "mm",
        "implants": implants,
        "metadata": {
            "timestamp": datetime.now().isoformat(),
            "source": "ios_scanbody_alignment",
            "n_implants": len(implants),
            "ios_case_id": ios_data.get("case_id"),
            "ios_frame": ios_data.get("frame"),
            "original_file": str(Path(args.input).resolve())
        }
    }
    
    # Save
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"\n✓ Saved implants_I.json: {output_path}")
    print(f"  {len(implants)} implants in I-frame (IOS coordinate system)")
    
    # Validation
    print(f"\nValidation:")
    for imp_id, imp_data in implants.items():
        axis_norm = np.linalg.norm(imp_data["axis_vector"])
        if not np.isclose(axis_norm, 1.0, atol=1e-6):
            print(f"  ⚠ Warning: Implant {imp_id} axis not unit (norm={axis_norm:.6f})")
    
    print(f"\n{'=' * 70}")
    print(f"✓ Conversion complete - ready for Phase 6 alignment")
    print(f"{'=' * 70}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
