#!/usr/bin/env python3
"""
Solve T_I_from_U: IOS Frame Alignment from Photogrammetry.

This tool computes the SE(3) transformation from photogrammetry U-frame to
IOS I-frame (exocad coordinate system) by matching implant centroids. Uses
correspondence-based Umeyama alignment with RMSE validation.

Workflow:
    1. Load implants_U.json (from photogrammetry)
    2. Load implants_I.json (from IOS/exocad scan body fit)
    3. Match implants by ID (marker_id or implant_id)
    4. Compute T_I_from_U using Umeyama alignment (closed-form SE(3))
    5. Transform implants to I-frame and validate RMSE
    6. Export T_I_from_U.json and implants_I_transformed.json

Usage:
    python tools/solve_T_I_from_U.py \\
        --implants-u runs/case001/implants_U.json \\
        --implants-i data/case001_ios/implants_exocad.json \\
        --output runs/case001/T_I_from_U.json \\
        --rmse-threshold 0.2

Quality Gates:
    - Minimum 3 implant correspondences required
    - RMSE < 0.2mm for clinical applications
    - All matched implants must have consistent marker_id

Author: Phase 6 IOS Integration Pipeline
"""

import argparse
import json
import sys
from pathlib import Path
from datetime import datetime
from typing import Dict, Tuple, Optional
import numpy as np

# Import alignment from existing transforms module
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
from transforms import compute_alignment, SE3Transform


def load_implants(json_path: str) -> Tuple[Dict, np.ndarray, np.ndarray, list]:
    """Load implant data from JSON file.
    
    Args:
        json_path: Path to implants JSON file
        
    Returns:
        Tuple of (full_data_dict, centroids_array, axes_array, implant_ids)
        - centroids_array: (N, 3) positions in mm
        - axes_array: (N, 3) unit axis vectors
        - implant_ids: list of N implant ID strings
        
    Raises:
        FileNotFoundError: If json_path doesn't exist
        ValueError: If JSON format is invalid
    """
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # Validate schema
    if data.get("units") != "mm":
        raise ValueError(f"Expected mm units, got {data.get('units')}")
    
    if "implants" not in data:
        raise ValueError("Missing 'implants' field in JSON")
    
    # Extract centroids and axes in sorted order
    implant_ids = sorted(data["implants"].keys(), key=int)
    
    centroids = []
    axes = []
    
    for implant_id in implant_ids:
        implant = data["implants"][implant_id]
        
        if "centroid_mm" not in implant:
            raise ValueError(f"Implant {implant_id} missing 'centroid_mm'")
        
        if "axis_vector" not in implant:
            raise ValueError(f"Implant {implant_id} missing 'axis_vector'")
        
        centroids.append(implant["centroid_mm"])
        axes.append(implant["axis_vector"])
    
    centroids_array = np.array(centroids)
    axes_array = np.array(axes)
    
    return data, centroids_array, axes_array, implant_ids


def match_implants(
    implants_u_data: Dict,
    implants_i_data: Dict
) -> Tuple[list, list, list]:
    """Match implants between U-frame and I-frame by marker_id.
    
    Args:
        implants_u_data: Full JSON dict from implants_U.json
        implants_i_data: Full JSON dict from implants_I.json
        
    Returns:
        Tuple of (u_ids, i_ids, marker_ids) for matched implants
        - u_ids: list of implant IDs in U-frame
        - i_ids: list of implant IDs in I-frame
        - marker_ids: list of corresponding marker IDs
        
    Raises:
        ValueError: If no matching implants found
    """
    # Build lookup tables: marker_id -> implant_id
    u_marker_to_id = {}
    for implant_id, implant in implants_u_data["implants"].items():
        marker_id = implant.get("marker_id")
        if marker_id is not None:
            u_marker_to_id[marker_id] = implant_id
    
    i_marker_to_id = {}
    for implant_id, implant in implants_i_data["implants"].items():
        marker_id = implant.get("marker_id")
        if marker_id is not None:
            i_marker_to_id[marker_id] = implant_id
    
    # Find common marker IDs
    common_markers = sorted(set(u_marker_to_id.keys()) & set(i_marker_to_id.keys()))
    
    if len(common_markers) == 0:
        raise ValueError("No matching implants found between U-frame and I-frame")
    
    # Build matched lists
    u_ids = [u_marker_to_id[m] for m in common_markers]
    i_ids = [i_marker_to_id[m] for m in common_markers]
    
    return u_ids, i_ids, common_markers


def compute_alignment_from_implants(
    centroids_u: np.ndarray,
    centroids_i: np.ndarray,
    allow_scaling: bool = False
) -> SE3Transform:
    """Compute T_I_from_U using Umeyama alignment.
    
    Args:
        centroids_u: (N, 3) implant centroids in U-frame (mm)
        centroids_i: (N, 3) implant centroids in I-frame (mm)
        allow_scaling: If True, use Sim(3) (similarity); else SE(3) (rigid)
        
    Returns:
        SE3Transform object representing T_I_from_U
        
    Raises:
        ValueError: If insufficient correspondences (N < 3)
    """
    if len(centroids_u) < 3:
        raise ValueError(f"Need at least 3 correspondences, got {len(centroids_u)}")
    
    # Convert arrays to dict format expected by compute_alignment
    src_dict = {str(i): centroids_u[i] for i in range(len(centroids_u))}
    dst_dict = {str(i): centroids_i[i] for i in range(len(centroids_i))}
    
    # Use transforms.compute_alignment (Umeyama algorithm)
    # Set high failure threshold to let caller validate RMSE
    transform, rmse, residuals = compute_alignment(
        src_dict,
        dst_dict,
        estimate_scale=allow_scaling,
        rmse_fail_threshold=1000.0  # Disable internal check, validate externally
    )
    
    return transform


def transform_implants_to_i_frame(
    implants_u_data: Dict,
    T_I_from_U: SE3Transform,
    matched_u_ids: list
) -> Dict:
    """Transform matched implants from U-frame to I-frame.
    
    Args:
        implants_u_data: Full JSON dict from implants_U.json
        T_I_from_U: SE3 transform from U to I
        matched_u_ids: List of implant IDs to transform
        
    Returns:
        New implants dict in I-frame with transformed centroids and axes
    """
    R = T_I_from_U.R
    t = T_I_from_U.t.flatten()  # Flatten (3,1) to (3,) for broadcasting
    
    transformed_implants = {}
    
    for implant_id in matched_u_ids:
        implant_u = implants_u_data["implants"][implant_id]
        
        # Transform centroid
        centroid_u = np.array(implant_u["centroid_mm"])
        centroid_i = R @ centroid_u + t
        
        # Transform axis (rotation only, no translation)
        axis_u = np.array(implant_u["axis_vector"])
        axis_i = R @ axis_u
        
        # Renormalize axis (should already be unit, but ensure)
        axis_i = axis_i / np.linalg.norm(axis_i)
        
        # Copy implant data with transformed pose
        transformed_implants[implant_id] = {
            "centroid_mm": centroid_i.tolist(),
            "axis_vector": axis_i.tolist(),
            "marker_id": implant_u.get("marker_id"),
            "reprojection_error_px": implant_u.get("reprojection_error_px", 0.0),
            "source_frame": "U"
        }
    
    return transformed_implants


def compute_rmse(
    centroids_i_transformed: np.ndarray,
    centroids_i_target: np.ndarray
) -> Tuple[float, np.ndarray]:
    """Compute RMSE and per-point residuals between transformed and target.
    
    Args:
        centroids_i_transformed: (N, 3) transformed centroids in I-frame
        centroids_i_target: (N, 3) target centroids from IOS in I-frame
        
    Returns:
        Tuple of (rmse_mm, residuals_array)
        - rmse_mm: Root mean square error in mm
        - residuals_array: (N,) array of per-point distances in mm
    """
    residuals = np.linalg.norm(centroids_i_transformed - centroids_i_target, axis=1)
    rmse = np.sqrt(np.mean(residuals**2))
    
    return rmse, residuals


def main():
    parser = argparse.ArgumentParser(
        description="Compute T_I_from_U transformation for IOS alignment",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic alignment with default 0.2mm threshold
    python tools/solve_T_I_from_U.py \\
        --implants-u runs/case001/implants_U.json \\
        --implants-i data/ios/implants_exocad.json \\
        --output runs/case001/T_I_from_U.json
    
    # Relaxed threshold for low-quality IOS scans
    python tools/solve_T_I_from_U.py \\
        --implants-u runs/case001/implants_U.json \\
        --implants-i data/ios/implants_exocad.json \\
        --output runs/case001/T_I_from_U.json \\
        --rmse-threshold 0.5
        """
    )
    
    parser.add_argument(
        "--implants-u",
        type=str,
        required=True,
        help="Path to implants_U.json (photogrammetry output)"
    )
    
    parser.add_argument(
        "--implants-i",
        type=str,
        required=True,
        help="Path to implants_I.json (IOS/exocad scan body positions)"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Path to save T_I_from_U.json"
    )
    
    parser.add_argument(
        "--rmse-threshold",
        type=float,
        default=0.2,
        help="Maximum allowable RMSE in mm (default: 0.2mm)"
    )
    
    parser.add_argument(
        "--allow-scaling",
        action="store_true",
        help="Allow scale in alignment (Sim3 instead of SE3)"
    )
    
    parser.add_argument(
        "--export-transformed",
        action="store_true",
        help="Export transformed implants_I.json alongside T_I_from_U.json"
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("Phase 6: Solve T_I_from_U (IOS Frame Alignment)")
    print("=" * 70)
    
    # Load implants
    print(f"\n[1/5] Loading implants...")
    print(f"  U-frame (photogrammetry): {args.implants_u}")
    print(f"  I-frame (IOS/exocad):     {args.implants_i}")
    
    u_data, centroids_u_all, axes_u_all, u_ids_all = load_implants(args.implants_u)
    i_data, centroids_i_all, axes_i_all, i_ids_all = load_implants(args.implants_i)
    
    print(f"  Loaded {len(u_ids_all)} implants from U-frame")
    print(f"  Loaded {len(i_ids_all)} implants from I-frame")
    
    # Match implants
    print(f"\n[2/5] Matching implants by marker_id...")
    matched_u_ids, matched_i_ids, marker_ids = match_implants(u_data, i_data)
    
    print(f"  Found {len(marker_ids)} matched correspondences:")
    for u_id, i_id, marker_id in zip(matched_u_ids, matched_i_ids, marker_ids):
        print(f"    U-implant {u_id} ↔ I-implant {i_id} (marker {marker_id})")
    
    if len(marker_ids) < 3:
        print(f"\n✗ ERROR: Need at least 3 correspondences, got {len(marker_ids)}")
        sys.exit(1)
    
    # Extract matched centroids
    centroids_u_matched = np.array([
        u_data["implants"][u_id]["centroid_mm"] for u_id in matched_u_ids
    ])
    centroids_i_matched = np.array([
        i_data["implants"][i_id]["centroid_mm"] for i_id in matched_i_ids
    ])
    
    # Compute alignment
    print(f"\n[3/5] Computing alignment...")
    transform_type = "Sim(3)" if args.allow_scaling else "SE(3)"
    print(f"  Algorithm: Umeyama {transform_type}")
    print(f"  Correspondences: {len(marker_ids)}")
    
    T_I_from_U = compute_alignment_from_implants(
        centroids_u_matched,
        centroids_i_matched,
        allow_scaling=args.allow_scaling
    )
    
    print(f"\n  Computed T_I_from_U:")
    print(f"    R = \n{T_I_from_U.R}")
    print(f"    t = {T_I_from_U.t}")
    if args.allow_scaling:
        print(f"    s = {T_I_from_U.s:.6f}")
    
    # Transform and validate
    print(f"\n[4/5] Validating alignment...")
    centroids_i_transformed = T_I_from_U.apply(centroids_u_matched)
    
    rmse, residuals = compute_rmse(centroids_i_transformed, centroids_i_matched)
    
    print(f"  RMSE: {rmse:.4f} mm")
    print(f"  Max residual: {np.max(residuals):.4f} mm")
    print(f"  Per-implant residuals:")
    for u_id, marker_id, residual in zip(matched_u_ids, marker_ids, residuals):
        print(f"    Implant {u_id} (marker {marker_id}): {residual:.4f} mm")
    
    # Check threshold
    if rmse > args.rmse_threshold:
        print(f"\n✗ FAIL: RMSE {rmse:.4f}mm exceeds threshold {args.rmse_threshold}mm")
        print(f"  This may indicate:")
        print(f"    - Incorrect implant correspondences")
        print(f"    - Poor IOS scan quality")
        print(f"    - Scan body misalignment")
        print(f"  Consider:")
        print(f"    - Verifying marker_id matches between datasets")
        print(f"    - Increasing --rmse-threshold if clinically acceptable")
        print(f"    - Re-scanning with better scan body seating")
        sys.exit(1)
    
    print(f"\n✓ PASS: RMSE within {args.rmse_threshold}mm threshold")
    
    # Export
    print(f"\n[5/5] Exporting results...")
    
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Build output JSON
    output_data = {
        "transform": "T_I_from_U",
        "description": "SE(3) transformation from photogrammetry U-frame to IOS I-frame",
        "R": T_I_from_U.R.tolist(),
        "t": T_I_from_U.t.tolist(),
        "metadata": {
            "timestamp": datetime.now().isoformat(),
            "n_correspondences": len(marker_ids),
            "matched_marker_ids": marker_ids,
            "rmse_mm": float(rmse),
            "max_residual_mm": float(np.max(residuals)),
            "rmse_threshold_mm": args.rmse_threshold,
            "allow_scaling": args.allow_scaling,
            "per_implant_residuals_mm": {
                str(u_id): float(res) for u_id, res in zip(matched_u_ids, residuals)
            },
            "source_u": str(Path(args.implants_u).resolve()),
            "source_i": str(Path(args.implants_i).resolve())
        }
    }
    
    if args.allow_scaling:
        output_data["s"] = float(T_I_from_U.s)
    
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"  Saved T_I_from_U.json: {output_path}")
    
    # Optional: export transformed implants
    if args.export_transformed:
        transformed_implants = transform_implants_to_i_frame(
            u_data,
            T_I_from_U,
            matched_u_ids
        )
        
        transformed_data = {
            "frame": "I",
            "units": "mm",
            "implants": transformed_implants,
            "metadata": {
                "timestamp": datetime.now().isoformat(),
                "source": "photogrammetry_transformed",
                "n_implants": len(transformed_implants),
                "transform_applied": "T_I_from_U",
                "original_source": str(Path(args.implants_u).resolve())
            }
        }
        
        transformed_path = output_path.parent / "implants_I_transformed.json"
        with open(transformed_path, 'w') as f:
            json.dump(transformed_data, f, indent=2)
        
        print(f"  Saved transformed implants: {transformed_path}")
    
    print(f"\n{'=' * 70}")
    print(f"✓ Phase 6 IOS alignment complete")
    print(f"  RMSE: {rmse:.4f} mm")
    print(f"  Correspondences: {len(marker_ids)}")
    print(f"{'=' * 70}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
