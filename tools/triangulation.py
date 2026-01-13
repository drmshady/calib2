"""
Triangulation utilities for 3D point reconstruction.

Implements DLT (Direct Linear Transform) with Levenberg-Marquardt refinement
for triangulating 3D points from 2D observations across multiple views.

Functions:
    triangulate_point_dlt: Linear triangulation via DLT
    refine_point_lm: Nonlinear refinement via Levenberg-Marquardt
    triangulate_point: DLT + LM pipeline
    triangulate_points: Batch triangulation with outlier rejection
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from scipy.optimize import least_squares


def triangulate_point_dlt(
    observations: np.ndarray,
    projection_matrices: np.ndarray
) -> np.ndarray:
    """Triangulate a 3D point using Direct Linear Transform (DLT).
    
    Solves the linear system A @ X = 0 where X is the homogeneous 3D point.
    Each observation contributes 2 equations to the system.
    
    Args:
        observations: Nx2 array of 2D image points (pixels)
        projection_matrices: Nx3x4 array of camera projection matrices (P = K @ [R|t])
        
    Returns:
        3D point coordinates (3,) in world frame (mm)
        
    Reference:
        Hartley & Zisserman (2004), "Multiple View Geometry", Section 12.2
    """
    n_views = observations.shape[0]
    A = np.zeros((2 * n_views, 4), dtype=np.float64)
    
    for i in range(n_views):
        x, y = observations[i]
        P = projection_matrices[i]
        
        # x * P[2,:] - P[0,:]
        A[2*i] = x * P[2, :] - P[0, :]
        
        # y * P[2,:] - P[1,:]
        A[2*i + 1] = y * P[2, :] - P[1, :]
    
    # Solve via SVD: A @ X = 0
    _, _, Vt = np.linalg.svd(A)
    X_homogeneous = Vt[-1, :]
    
    # Convert from homogeneous to 3D
    X = X_homogeneous[:3] / X_homogeneous[3]
    
    return X


def project_point(
    point_3d: np.ndarray,
    projection_matrix: np.ndarray
) -> np.ndarray:
    """Project 3D point to 2D image coordinates.
    
    Args:
        point_3d: 3D point coordinates (3,) in world frame (mm)
        projection_matrix: 3x4 camera projection matrix (P = K @ [R|t])
        
    Returns:
        2D image coordinates (2,) in pixels
    """
    point_homogeneous = np.append(point_3d, 1.0)
    projected = projection_matrix @ point_homogeneous
    return projected[:2] / projected[2]


def compute_reprojection_errors(
    point_3d: np.ndarray,
    observations: np.ndarray,
    projection_matrices: np.ndarray
) -> np.ndarray:
    """Compute reprojection errors for all views.
    
    Args:
        point_3d: 3D point coordinates (3,) in world frame (mm)
        observations: Nx2 array of observed 2D points (pixels)
        projection_matrices: Nx3x4 array of projection matrices
        
    Returns:
        N-array of reprojection errors (Euclidean distance in pixels)
    """
    n_views = observations.shape[0]
    errors = np.zeros(n_views)
    
    for i in range(n_views):
        projected = project_point(point_3d, projection_matrices[i])
        errors[i] = np.linalg.norm(observations[i] - projected)
    
    return errors


def reprojection_residuals(
    point_3d: np.ndarray,
    observations: np.ndarray,
    projection_matrices: np.ndarray
) -> np.ndarray:
    """Compute residual vector for Levenberg-Marquardt optimization.
    
    Args:
        point_3d: 3D point coordinates (3,) in world frame (mm)
        observations: Nx2 array of observed 2D points (pixels)
        projection_matrices: Nx3x4 array of projection matrices
        
    Returns:
        2N-array of residuals [dx_0, dy_0, dx_1, dy_1, ...]
    """
    n_views = observations.shape[0]
    residuals = np.zeros(2 * n_views)
    
    for i in range(n_views):
        projected = project_point(point_3d, projection_matrices[i])
        residuals[2*i:2*i+2] = observations[i] - projected
    
    return residuals


def refine_point_lm(
    point_3d_init: np.ndarray,
    observations: np.ndarray,
    projection_matrices: np.ndarray,
    max_iterations: int = 100
) -> np.ndarray:
    """Refine 3D point using Levenberg-Marquardt optimization.
    
    Minimizes sum of squared reprojection errors.
    
    Args:
        point_3d_init: Initial 3D point estimate (3,) from DLT (mm)
        observations: Nx2 array of 2D observations (pixels)
        projection_matrices: Nx3x4 array of projection matrices
        max_iterations: Maximum LM iterations
        
    Returns:
        Refined 3D point coordinates (3,) in mm
    """
    result = least_squares(
        fun=reprojection_residuals,
        x0=point_3d_init,
        args=(observations, projection_matrices),
        method='lm',
        max_nfev=max_iterations
    )
    
    return result.x


def triangulate_point(
    observations: np.ndarray,
    projection_matrices: np.ndarray,
    refine: bool = True,
    outlier_threshold_px: float = 5.0
) -> Tuple[np.ndarray, float, np.ndarray]:
    """Triangulate 3D point with DLT initialization and optional LM refinement.
    
    Args:
        observations: Nx2 array of 2D image points (pixels), N >= 2
        projection_matrices: Nx3x4 array of camera projection matrices
        refine: If True, apply Levenberg-Marquardt refinement after DLT
        outlier_threshold_px: Reprojection error threshold for outlier detection
        
    Returns:
        Tuple of (point_3d, mean_error, errors):
            point_3d: Triangulated 3D point (3,) in mm
            mean_error: Mean reprojection error in pixels
            errors: Per-view reprojection errors (N,) in pixels
            
    Raises:
        ValueError: If fewer than 2 views provided
    """
    if observations.shape[0] < 2:
        raise ValueError(f"Need at least 2 views, got {observations.shape[0]}")
    
    # DLT initialization
    point_3d = triangulate_point_dlt(observations, projection_matrices)
    
    # Levenberg-Marquardt refinement
    if refine:
        point_3d = refine_point_lm(point_3d, observations, projection_matrices)
    
    # Compute reprojection errors
    errors = compute_reprojection_errors(point_3d, observations, projection_matrices)
    mean_error = np.mean(errors)
    
    # Warn if high errors detected
    if mean_error > outlier_threshold_px:
        print(
            f"WARNING: Mean reprojection error {mean_error:.2f} px exceeds "
            f"threshold {outlier_threshold_px:.2f} px"
        )
    
    return point_3d, mean_error, errors


def triangulate_points(
    point_observations: Dict[str, List[Tuple[int, np.ndarray]]],
    projection_matrices: Dict[int, np.ndarray],
    min_views: int = 2,
    outlier_threshold_px: float = 5.0,
    refine: bool = True
) -> Dict[str, Dict]:
    """Triangulate multiple 3D points with outlier rejection.
    
    Args:
        point_observations: Dict mapping point IDs to list of (view_id, pixel_coords)
                           Example: {"1_TL": [(0, [x0, y0]), (1, [x1, y1])]}
        projection_matrices: Dict mapping view_id to 3x4 projection matrix
        min_views: Minimum number of views required per point
        outlier_threshold_px: Reprojection error threshold for outlier rejection
        refine: If True, apply LM refinement
        
    Returns:
        Dict mapping point IDs to triangulation results:
            {
                "point_id": {
                    "position_mm": [x, y, z],
                    "n_views": int,
                    "mean_reprojection_error_px": float,
                    "max_reprojection_error_px": float,
                    "view_errors_px": {view_id: error}
                }
            }
            
    Example:
        >>> observations = {
        ...     "1_TL": [(0, [100.0, 200.0]), (1, [110.0, 210.0])],
        ...     "1_TR": [(0, [150.0, 200.0]), (1, [160.0, 210.0])]
        ... }
        >>> P_matrices = {0: P0, 1: P1}  # 3x4 projection matrices
        >>> results = triangulate_points(observations, P_matrices)
    """
    results = {}
    
    for point_id, obs_list in point_observations.items():
        # Filter by minimum views
        if len(obs_list) < min_views:
            print(
                f"Skipping {point_id}: only {len(obs_list)} views "
                f"(min {min_views} required)"
            )
            continue
        
        # Build arrays for this point
        view_ids = [view_id for view_id, _ in obs_list]
        observations = np.array([pixel_coords for _, pixel_coords in obs_list])
        P_matrices = np.array([projection_matrices[vid] for vid in view_ids])
        
        try:
            # Triangulate
            point_3d, mean_error, errors = triangulate_point(
                observations, P_matrices, refine, outlier_threshold_px
            )
            
            # Check for outliers
            outlier_mask = errors > outlier_threshold_px
            if np.any(outlier_mask):
                outlier_views = [view_ids[i] for i in np.where(outlier_mask)[0]]
                print(
                    f"WARNING: {point_id} has outliers in views {outlier_views} "
                    f"(errors: {errors[outlier_mask]})"
                )
                
                # Re-triangulate without outliers
                if np.sum(~outlier_mask) >= min_views:
                    inlier_indices = np.where(~outlier_mask)[0]
                    view_ids = [view_ids[i] for i in inlier_indices]
                    observations = observations[inlier_indices]
                    P_matrices = P_matrices[inlier_indices]
                    
                    point_3d, mean_error, errors = triangulate_point(
                        observations, P_matrices, refine, outlier_threshold_px
                    )
                    print(f"Re-triangulated {point_id} with {len(view_ids)} inliers")
            
            # Store result
            results[point_id] = {
                "position_mm": point_3d.tolist(),
                "n_views": len(view_ids),
                "mean_reprojection_error_px": float(mean_error),
                "max_reprojection_error_px": float(np.max(errors)),
                "view_errors_px": dict(zip(view_ids, errors.tolist()))
            }
            
        except Exception as e:
            print(f"ERROR triangulating {point_id}: {e}")
            continue
    
    return results


def build_projection_matrix(
    K: np.ndarray,
    R: np.ndarray,
    t: np.ndarray
) -> np.ndarray:
    """Build 3x4 projection matrix from intrinsics and extrinsics.
    
    Args:
        K: 3x3 camera intrinsic matrix
        R: 3x3 rotation matrix (world to camera)
        t: 3x1 or (3,) translation vector (mm)
        
    Returns:
        3x4 projection matrix P = K @ [R | t]
    """
    t = np.array(t).reshape(3, 1)
    Rt = np.hstack([R, t])
    P = K @ Rt
    return P


def cameras_to_projection_matrices(
    cameras: Dict[int, Dict],
    K: np.ndarray
) -> Dict[int, np.ndarray]:
    """Convert camera poses to projection matrices.
    
    Args:
        cameras: Dict mapping view_id to {"R": 3x3, "t": 3x1} in world frame
        K: 3x3 camera intrinsic matrix
        
    Returns:
        Dict mapping view_id to 3x4 projection matrix
    """
    P_matrices = {}
    
    for view_id, camera in cameras.items():
        R = np.array(camera["R"])
        t = np.array(camera["t"])
        P_matrices[view_id] = build_projection_matrix(K, R, t)
    
    return P_matrices
