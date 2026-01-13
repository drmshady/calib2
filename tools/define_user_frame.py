"""
Define User (U) frame from reference plate alignment.

Computes the transform T_U_from_L that aligns the L-frame (solver output)
to the U-frame (canonical reference frame) using AprilTag corner correspondences
from a reference plate fixture.

Functions:
    define_user_frame: Compute T_U_from_L from refpoints_L.json and reference plate
    apply_user_frame: Transform all points from L-frame to U-frame
"""

import numpy as np
import json
from pathlib import Path
from typing import Dict, Tuple, Optional
from src.transforms import compute_alignment, SE3Transform, Sim3Transform
from tools.bundle_adjustment import load_refpoints_L


def define_user_frame(
    refpoints_L_path: str,
    reference_plate_path: str,
    estimate_scale: bool = False,
    rmse_warn_threshold: float = 0.1,
    rmse_fail_threshold: float = 0.5,
    output_path: Optional[str] = None
) -> Tuple[SE3Transform, float]:
    """Compute T_U_from_L transform via reference plate alignment.
    
    Aligns triangulated L-frame points to known U-frame reference geometry
    using Umeyama algorithm. Uses AprilTag corner correspondences from
    reference plate (IDs 1-4, 16 corners total).
    
    Args:
        refpoints_L_path: Path to refpoints_L.json (triangulated L-frame points)
        reference_plate_path: Path to reference_plate_4tags.json (known U-frame geometry)
        estimate_scale: If True, use Sim(3); if False, use SE(3)
        rmse_warn_threshold: Warning threshold in mm
        rmse_fail_threshold: Failure threshold in mm
        output_path: Optional path to save T_U_from_L.json
        
    Returns:
        Tuple of (T_U_from_L, rmse):
            T_U_from_L: SE3Transform or Sim3Transform from L to U
            rmse: Alignment RMSE in mm
            
    Raises:
        ValueError: If no common points found or RMSE exceeds threshold
        
    Example:
        >>> T_U_from_L, rmse = define_user_frame(
        ...     "runs/test01/refpoints_L.json",
        ...     "calib/fixtures/reference_plate_4tags.json",
        ...     output_path="runs/test01/T_U_from_L.json"
        ... )
        >>> print(f"Alignment RMSE: {rmse:.3f} mm")
    """
    # Load L-frame points (triangulated)
    points_L, metadata_L = load_refpoints_L(refpoints_L_path)
    
    # Load U-frame reference geometry
    with open(reference_plate_path, 'r') as f:
        reference_data = json.load(f)
    
    if reference_data.get("frame") != "U":
        raise ValueError(f"Reference plate must be in U-frame, got {reference_data.get('frame')}")
    
    points_U = {
        point_id: np.array(coords, dtype=np.float64)
        for point_id, coords in reference_data["points"].items()
    }
    
    # Find common points (should be reference plate tags: 1_TL, 1_TR, ..., 4_BL)
    common_ids = set(points_L.keys()) & set(points_U.keys())
    
    if not common_ids:
        raise ValueError(
            f"No common points between L-frame ({len(points_L)} points) "
            f"and reference plate ({len(points_U)} points)"
        )
    
    print(f"Found {len(common_ids)} common reference points: {sorted(common_ids)}")
    
    # Compute alignment
    T_U_from_L, rmse, residuals = compute_alignment(
        src_points=points_L,
        dst_points=points_U,
        estimate_scale=estimate_scale,
        rmse_warn_threshold=rmse_warn_threshold,
        rmse_fail_threshold=rmse_fail_threshold
    )
    
    # Set frame labels
    T_U_from_L.source_frame = "L"
    T_U_from_L.target_frame = "U"
    
    # Log per-point residuals
    print("\nPer-point alignment residuals:")
    for point_id in sorted(residuals.keys()):
        print(f"  {point_id}: {residuals[point_id]:.4f} mm")
    
    print(f"\nOverall alignment RMSE: {rmse:.4f} mm")
    
    # Save transform if requested
    if output_path:
        T_U_from_L.save(
            output_path,
            rmse_mm=rmse,
            n_points=len(common_ids),
            reference_plate=reference_plate_path,
            refpoints_L_source=refpoints_L_path,
            residuals_mm=residuals
        )
        print(f"Saved T_U_from_L to {output_path}")
    
    return T_U_from_L, rmse


def apply_user_frame(
    refpoints_L_path: str,
    T_U_from_L_path: str,
    output_path: str
):
    """Transform all L-frame points to U-frame.
    
    Loads refpoints_L.json and T_U_from_L.json, applies transform,
    and exports refpoints_U.json.
    
    Args:
        refpoints_L_path: Path to refpoints_L.json
        T_U_from_L_path: Path to T_U_from_L.json
        output_path: Path to save refpoints_U.json
        
    Example:
        >>> apply_user_frame(
        ...     "runs/test01/refpoints_L.json",
        ...     "runs/test01/T_U_from_L.json",
        ...     "runs/test01/refpoints_U.json"
        ... )
    """
    # Load L-frame points
    points_L, metadata_L = load_refpoints_L(refpoints_L_path)
    
    # Load transform
    with open(T_U_from_L_path, 'r') as f:
        transform_data = json.load(f)
    
    if transform_data.get("transform_type") == "SE3":
        T_U_from_L = SE3Transform.load(T_U_from_L_path)
    elif transform_data.get("transform_type") == "Sim3":
        T_U_from_L = Sim3Transform.load(T_U_from_L_path)
    else:
        raise ValueError(f"Unknown transform type: {transform_data.get('transform_type')}")
    
    # Apply transform
    points_U = {}
    for point_id, point_L in points_L.items():
        point_U = T_U_from_L.apply(point_L.reshape(1, -1)).flatten()
        points_U[point_id] = point_U.tolist()
    
    # Build output
    data = {
        "frame": "U",
        "units": "mm",
        "points": points_U,
        "metadata": {
            "source": "refpoints_L.json transformed via T_U_from_L.json",
            "refpoints_L_source": refpoints_L_path,
            "transform_source": T_U_from_L_path,
            "n_points": len(points_U),
            **metadata_L  # Inherit triangulation metadata
        }
    }
    
    # Save
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Transformed {len(points_U)} points to U-frame")
    print(f"Saved to {output_path}")


def load_user_frame_transform(filepath: str) -> SE3Transform:
    """Load T_U_from_L transform from JSON file.
    
    Args:
        filepath: Path to T_U_from_L.json
        
    Returns:
        SE3Transform or Sim3Transform instance
    """
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    transform_type = data.get("transform_type")
    
    if transform_type == "SE3":
        return SE3Transform.load(filepath)
    elif transform_type == "Sim3":
        return Sim3Transform.load(filepath)
    else:
        raise ValueError(f"Unknown transform type: {transform_type}")


def validate_user_frame_definition(
    refpoints_L_path: str,
    reference_plate_path: str,
    T_U_from_L_path: str,
    max_residual_mm: float = 0.5
):
    """Validate that T_U_from_L correctly aligns reference plate.
    
    Re-computes alignment and checks that saved transform matches.
    
    Args:
        refpoints_L_path: Path to refpoints_L.json
        reference_plate_path: Path to reference_plate_4tags.json
        T_U_from_L_path: Path to T_U_from_L.json
        max_residual_mm: Maximum acceptable residual in mm
        
    Raises:
        ValueError: If validation fails
    """
    # Load saved transform
    T_saved = load_user_frame_transform(T_U_from_L_path)
    
    # Re-compute transform
    T_computed, rmse = define_user_frame(
        refpoints_L_path,
        reference_plate_path,
        estimate_scale=isinstance(T_saved, Sim3Transform)
    )
    
    # Check RMSE
    if rmse > max_residual_mm:
        raise ValueError(f"RMSE {rmse:.3f} mm exceeds threshold {max_residual_mm:.3f} mm")
    
    # Check matrices match
    T_saved_matrix = T_saved.to_matrix()
    T_computed_matrix = T_computed.to_matrix()
    
    matrix_diff = np.max(np.abs(T_saved_matrix - T_computed_matrix))
    if matrix_diff > 1e-6:
        raise ValueError(
            f"Saved transform differs from re-computed: max diff = {matrix_diff:.2e}"
        )
    
    print(f"✓ Validated T_U_from_L")
    print(f"  RMSE: {rmse:.4f} mm")
    print(f"  Matrix match: {matrix_diff:.2e}")


# CLI interface for standalone use
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 3:
        print("Usage: python define_user_frame.py <refpoints_L.json> <reference_plate.json> [output_T_U_from_L.json]")
        sys.exit(1)
    
    refpoints_L_path = sys.argv[1]
    reference_plate_path = sys.argv[2]
    output_path = sys.argv[3] if len(sys.argv) > 3 else None
    
    T_U_from_L, rmse = define_user_frame(
        refpoints_L_path,
        reference_plate_path,
        output_path=output_path
    )
    
    print(f"\n✓ Successfully computed T_U_from_L (RMSE: {rmse:.4f} mm)")
