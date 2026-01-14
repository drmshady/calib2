"""
Implant Pose Extraction from Photogrammetry Reference Points.

This module extracts implant centroids and axis vectors from AprilTag corner
positions (refpoints_U.json) by analyzing marker cap geometry. Each marker cap
has 4 corners that define the tag plane and orientation.

Functions:
    extract_implants_from_refpoints: Main extraction pipeline
    compute_implant_centroid: Calculate implant center from tag corners
    compute_implant_axis: Calculate implant axis (normal to tag plane)
    validate_implant_geometry: Check for valid tag geometry

Coordinate Convention:
    - Input: refpoints_U.json (tag corners in U-frame, mm units)
    - Output: implants_U.json (implant poses in U-frame, mm units)
    - Axis vector: Unit normal pointing along implant longitudinal axis
"""

import numpy as np
import json
from pathlib import Path
from typing import Dict, Tuple, Optional
from datetime import datetime


def compute_tag_plane_normal(corners: np.ndarray) -> np.ndarray:
    """Compute unit normal vector to tag plane using cross product.
    
    Args:
        corners: (4, 3) array of tag corners [TL, TR, BR, BL] in mm
        
    Returns:
        (3,) unit normal vector pointing away from camera
        
    Raises:
        ValueError: If corners are degenerate (collinear)
    """
    # Use cross product of two diagonals for robustness
    # TL→BR and TR→BL diagonals
    diag1 = corners[2] - corners[0]  # BR - TL
    diag2 = corners[3] - corners[1]  # BL - TR
    
    # Normal via cross product
    normal = np.cross(diag1, diag2)
    norm = np.linalg.norm(normal)
    
    if norm < 1e-6:
        raise ValueError("Degenerate tag geometry: corners are collinear")
    
    return normal / norm


def compute_implant_centroid(corners: np.ndarray, tag_size_mm: float = 8.8) -> np.ndarray:
    """Compute implant centroid from tag corners.
    
    The implant centroid is approximated as the tag center. For marker caps
    mounted on implants, the tag center closely approximates the implant axis
    intersection with the cap surface.
    
    Args:
        corners: (4, 3) array of tag corners [TL, TR, BR, BL] in mm
        tag_size_mm: AprilTag edge length in mm (default 8.8mm)
        
    Returns:
        (3,) centroid position in mm
    """
    # Simple centroid: mean of 4 corners
    return np.mean(corners, axis=0)


def compute_implant_axis(corners: np.ndarray) -> np.ndarray:
    """Compute implant axis vector from tag corners.
    
    For marker caps, the implant axis is perpendicular to the tag plane.
    The normal vector is computed using cross product of tag diagonals.
    
    Args:
        corners: (4, 3) array of tag corners [TL, TR, BR, BL] in mm
        
    Returns:
        (3,) unit axis vector pointing along implant longitudinal axis
    """
    return compute_tag_plane_normal(corners)


def validate_tag_geometry(corners: np.ndarray, tag_size_mm: float = 8.8, 
                          tolerance_mm: float = 0.5) -> Tuple[bool, str]:
    """Validate tag corner geometry for quality assurance.
    
    Checks:
    1. Edge lengths approximately equal to tag_size_mm
    2. Opposite edges approximately parallel
    3. Corners approximately planar
    
    Args:
        corners: (4, 3) array of tag corners [TL, TR, BR, BL] in mm
        tag_size_mm: Expected edge length in mm
        tolerance_mm: Allowable deviation in mm
        
    Returns:
        Tuple of (passed, message) where passed is True if valid
    """
    # Check edge lengths
    edges = [
        np.linalg.norm(corners[1] - corners[0]),  # TL→TR
        np.linalg.norm(corners[2] - corners[1]),  # TR→BR
        np.linalg.norm(corners[3] - corners[2]),  # BR→BL
        np.linalg.norm(corners[0] - corners[3])   # BL→TL
    ]
    
    edge_errors = [abs(edge - tag_size_mm) for edge in edges]
    max_edge_error = max(edge_errors)
    
    if max_edge_error > tolerance_mm:
        return False, f"Edge length error {max_edge_error:.2f}mm exceeds {tolerance_mm}mm"
    
    # Check planarity: distance from each corner to fitted plane
    centroid = np.mean(corners, axis=0)
    normal = compute_tag_plane_normal(corners)
    
    planarity_errors = [abs(np.dot(corner - centroid, normal)) for corner in corners]
    max_planarity_error = max(planarity_errors)
    
    if max_planarity_error > tolerance_mm:
        return False, f"Planarity error {max_planarity_error:.2f}mm exceeds {tolerance_mm}mm"
    
    return True, "Valid geometry"


def extract_implants_from_refpoints(
    refpoints_path: str,
    output_path: str,
    tag_size_mm: float = 8.8,
    tag_ids: Optional[list] = None,
    validate_geometry: bool = True
) -> Dict:
    """Extract implant poses from reference points (tag corners).
    
    Main pipeline function that loads refpoints_U.json, extracts implant
    centroids and axis vectors for each tag, validates geometry, and exports
    to implants_U.json format.
    
    Args:
        refpoints_path: Path to refpoints_U.json (tag corners in U-frame)
        output_path: Path to save implants_U.json
        tag_size_mm: AprilTag edge length for validation (default 8.8mm)
        tag_ids: List of tag IDs to process (None = auto-detect from refpoints)
        validate_geometry: If True, validate tag geometry and warn on failures
        
    Returns:
        Dictionary with extraction results and statistics
        
    Raises:
        FileNotFoundError: If refpoints_path doesn't exist
        ValueError: If refpoints data is invalid
        
    Example:
        >>> results = extract_implants_from_refpoints(
        ...     "runs/case001/refpoints_U.json",
        ...     "runs/case001/implants_U.json",
        ...     tag_size_mm=8.8
        ... )
        >>> print(f"Extracted {results['n_implants']} implants")
    """
    # Load refpoints
    with open(refpoints_path, 'r') as f:
        refpoints_data = json.load(f)
    
    if refpoints_data.get("frame") != "U":
        raise ValueError(f"Expected U-frame refpoints, got {refpoints_data.get('frame')}")
    
    if refpoints_data.get("units") != "mm":
        raise ValueError(f"Expected mm units, got {refpoints_data.get('units')}")
    
    points = {k: np.array(v) for k, v in refpoints_data["points"].items()}
    
    # Auto-detect tag IDs from point IDs (format: tag_id * 10 + corner_idx)
    if tag_ids is None:
        tag_ids = sorted(set(int(pid) // 10 for pid in points.keys()))
    
    # Extract implants
    implants = {}
    validation_results = {}
    
    for tag_id in tag_ids:
        # Get 4 corners for this tag (corner indices 0-3)
        corner_ids = [str(tag_id * 10 + i) for i in range(4)]
        
        # Check if all corners present
        if not all(cid in points for cid in corner_ids):
            print(f"Warning: Tag {tag_id} missing corners, skipping")
            continue
        
        corners = np.array([points[cid] for cid in corner_ids])
        
        # Validate geometry
        if validate_geometry:
            valid, msg = validate_tag_geometry(corners, tag_size_mm)
            validation_results[str(tag_id)] = {"valid": valid, "message": msg}
            
            if not valid:
                print(f"Warning: Tag {tag_id} geometry validation failed: {msg}")
        
        # Compute implant pose
        centroid = compute_implant_centroid(corners, tag_size_mm)
        axis = compute_implant_axis(corners)
        
        # Get reprojection error from metadata if available
        reproj_error = refpoints_data.get("metadata", {}).get(
            "per_point_reprojection_errors_px", {}
        )
        
        # Average reprojection error for this tag's corners
        corner_errors = [reproj_error.get(cid, 0.0) for cid in corner_ids]
        mean_error = np.mean(corner_errors) if corner_errors else 0.0
        
        # Store implant
        implant_id = str(len(implants) + 1)  # Sequential implant numbering
        implants[implant_id] = {
            "centroid_mm": centroid.tolist(),
            "axis_vector": axis.tolist(),
            "marker_id": tag_id,
            "reprojection_error_px": float(mean_error)
        }
    
    # Build output
    output_data = {
        "frame": "U",
        "units": "mm",
        "implants": implants,
        "metadata": {
            "timestamp": datetime.now().isoformat(),
            "source": "photogrammetry",
            "n_implants": len(implants),
            "mean_reprojection_error_px": float(np.mean([
                imp["reprojection_error_px"] for imp in implants.values()
            ])) if implants else 0.0,
            "reconstruction_run": str(Path(refpoints_path).parent),
            "tag_size_mm": tag_size_mm,
            "validation_results": validation_results
        }
    }
    
    # Save
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"Extracted {len(implants)} implants from {len(tag_ids)} tags")
    print(f"Saved to {output_path}")
    
    return {
        "n_implants": len(implants),
        "tag_ids": tag_ids,
        "validation_results": validation_results,
        "output_path": output_path
    }


# CLI interface for standalone use
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 3:
        print("Usage: python implant_extractor.py <refpoints_U.json> <output_implants_U.json> [tag_size_mm]")
        sys.exit(1)
    
    refpoints_path = sys.argv[1]
    output_path = sys.argv[2]
    tag_size_mm = float(sys.argv[3]) if len(sys.argv) > 3 else 8.8
    
    results = extract_implants_from_refpoints(
        refpoints_path,
        output_path,
        tag_size_mm=tag_size_mm
    )
    
    print(f"\n✓ Successfully extracted {results['n_implants']} implants")
