"""
Bundle adjustment utilities for camera pose and 3D structure refinement.

This module provides functions to export triangulated 3D points (refpoints_L.json)
from bundle adjustment results. The exported points represent the TRUE solver
output frame (L-frame) from triangulated 2D observations.

Note: This is a Phase 0 skeleton. Full bundle adjustment implementation is in Phase 3.

Functions:
    export_refpoints_L: Export triangulated 3D points to refpoints_L.json
"""

import numpy as np
import json
from pathlib import Path
from typing import Dict, List, Tuple
from datetime import datetime


def export_refpoints_L(
    triangulated_points: Dict[str, Dict],
    output_path: str,
    metadata: Dict = None
):
    """Export triangulated 3D points to refpoints_L.json.
    
    This represents the TRUE solver output frame (L-frame) from bundle adjustment.
    Points are triangulated from 2D observations using optimized camera poses.
    
    Args:
        triangulated_points: Dict mapping point IDs to triangulation results
                            (output from tools.triangulation.triangulate_points)
                            Format: {
                                "point_id": {
                                    "position_mm": [x, y, z],
                                    "n_views": int,
                                    "mean_reprojection_error_px": float,
                                    "max_reprojection_error_px": float,
                                    "view_errors_px": {view_id: error}
                                }
                            }
        output_path: Output JSON file path (typically runs/<dataset>/refpoints_L.json)
        metadata: Optional dict with additional metadata (e.g., dataset name, timestamp)
        
    Example:
        >>> from tools.triangulation import triangulate_points
        >>> triangulated = triangulate_points(observations, projection_matrices)
        >>> export_refpoints_L(triangulated, "runs/test01/refpoints_L.json")
    """
    # Build points dict (point_id -> [x, y, z])
    points = {}
    for point_id, result in triangulated_points.items():
        points[point_id] = result["position_mm"]
    
    # Build metadata
    reprojection_errors = {
        pid: result["mean_reprojection_error_px"]
        for pid, result in triangulated_points.items()
    }
    
    n_views_per_point = {
        pid: result["n_views"]
        for pid, result in triangulated_points.items()
    }
    
    # Compute overall statistics
    mean_error = np.mean(list(reprojection_errors.values()))
    max_error = np.max(list(reprojection_errors.values()))
    
    # Build output structure
    data = {
        "frame": "L",
        "units": "mm",
        "points": points,
        "metadata": {
            "triangulation_method": "DLT+LM",
            "mean_reprojection_error_px": float(mean_error),
            "max_reprojection_error_px": float(max_error),
            "n_points": len(points),
            "n_views_per_point": n_views_per_point,
            "per_point_reprojection_errors_px": reprojection_errors,
            "timestamp": datetime.now().isoformat()
        }
    }
    
    # Merge custom metadata
    if metadata:
        data["metadata"].update(metadata)
    
    # Save to file
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Exported {len(points)} points to {output_path}")
    print(f"  Mean reprojection error: {mean_error:.3f} px")
    print(f"  Max reprojection error: {max_error:.3f} px")


def load_refpoints_L(filepath: str) -> Tuple[Dict[str, np.ndarray], Dict]:
    """Load triangulated 3D points from refpoints_L.json.
    
    Args:
        filepath: Path to refpoints_L.json file
        
    Returns:
        Tuple of (points, metadata):
            points: Dict mapping point IDs to 3D positions (Nx3 numpy arrays) in mm
            metadata: Dict with triangulation metadata
            
    Raises:
        ValueError: If file format is invalid or frame is not "L"
    """
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    # Validate format
    if data.get("frame") != "L":
        raise ValueError(f"Expected L-frame, got {data.get('frame')}")
    
    if data.get("units") != "mm":
        raise ValueError(f"Expected units 'mm', got {data.get('units')}")
    
    # Convert points to numpy arrays
    points = {
        point_id: np.array(coords, dtype=np.float64)
        for point_id, coords in data["points"].items()
    }
    
    metadata = data.get("metadata", {})
    
    return points, metadata


def validate_refpoints_L(filepath: str, max_reprojection_error_px: float = 5.0):
    """Validate refpoints_L.json quality gates.
    
    Args:
        filepath: Path to refpoints_L.json file
        max_reprojection_error_px: Maximum acceptable reprojection error
        
    Raises:
        ValueError: If validation fails
    """
    points, metadata = load_refpoints_L(filepath)
    
    # Check required metadata
    if "mean_reprojection_error_px" not in metadata:
        raise ValueError("Missing mean_reprojection_error_px in metadata")
    
    mean_error = metadata["mean_reprojection_error_px"]
    max_error = metadata.get("max_reprojection_error_px", mean_error)
    
    # Validate reprojection errors
    if mean_error > max_reprojection_error_px:
        raise ValueError(
            f"Mean reprojection error {mean_error:.3f} px exceeds threshold "
            f"{max_reprojection_error_px:.3f} px"
        )
    
    # Check minimum number of points
    n_points = len(points)
    if n_points < 4:
        raise ValueError(f"Too few points: {n_points} < 4 (minimum for reference plate)")
    
    # Validate point IDs (should be tag corner format)
    for point_id in points.keys():
        if '_' not in point_id:
            print(f"WARNING: Point ID '{point_id}' not in 'tagID_corner' format")
    
    print(f"✓ Validated {filepath}")
    print(f"  {n_points} points, mean error {mean_error:.3f} px, max error {max_error:.3f} px")


# Legacy model coordinate support (deprecated)
def export_geometry_L_MODEL(
    model_points: Dict[str, np.ndarray],
    output_path: str,
    deprecation_warning: bool = True
):
    """Export CAD model coordinates (DEPRECATED, use refpoints_L.json instead).
    
    This function is retained for backward compatibility but is deprecated.
    Use export_refpoints_L() with triangulated points instead.
    
    Args:
        model_points: Dict mapping point IDs to 3D positions from CAD model
        output_path: Output JSON file path
        deprecation_warning: If True, print deprecation warning
    """
    if deprecation_warning:
        print("=" * 60)
        print("WARNING: export_geometry_L_MODEL is DEPRECATED")
        print("Use export_refpoints_L() with triangulated points instead.")
        print("CAD model coordinates do NOT represent true solver output.")
        print("=" * 60)
    
    data = {
        "frame": "L_MODEL",
        "units": "mm",
        "points": {pid: pos.tolist() for pid, pos in model_points.items()},
        "metadata": {
            "source": "CAD_model",
            "deprecated": True,
            "use_instead": "refpoints_L.json from triangulation",
            "timestamp": datetime.now().isoformat()
        }
    }
    
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
