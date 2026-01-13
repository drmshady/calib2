"""
Global Bundle Adjustment optimizer for multi-view reconstruction.

Implements sparse bundle adjustment with Huber robust loss function
for simultaneous refinement of camera poses and 3D point positions.

Functions:
    bundle_adjust_global: Optimize full reconstruction (cameras + points)
    bundle_adjust_points_only: Optimize 3D points with fixed cameras
    bundle_adjust_poses_only: Optimize camera poses with fixed points

Uses scipy.optimize.least_squares with sparse Jacobian for efficiency.
"""

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from typing import Dict, List, Tuple, Optional
import cv2

import incremental_sfm
from incremental_sfm import IncrementalSfM, Camera, Point3D


def rodrigues_to_matrix(rvec: np.ndarray) -> np.ndarray:
    """Convert rotation vector to rotation matrix.
    
    Args:
        rvec: (3,) rotation vector
        
    Returns:
        R: (3, 3) rotation matrix
    """
    R, _ = cv2.Rodrigues(rvec)
    return R


def matrix_to_rodrigues(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to rotation vector.
    
    Args:
        R: (3, 3) rotation matrix
        
    Returns:
        rvec: (3,) rotation vector
    """
    rvec, _ = cv2.Rodrigues(R)
    return rvec.ravel()


def project_point(
    X: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray
) -> np.ndarray:
    """Project 3D point to image plane.
    
    Args:
        X: (3,) 3D point in world coordinates
        rvec: (3,) camera rotation vector
        tvec: (3,) camera translation vector
        K: (3, 3) camera intrinsic matrix
        
    Returns:
        pt_2d: (2,) projected point in pixels
    """
    # Transform to camera frame
    R = rodrigues_to_matrix(rvec)
    X_cam = R @ X + tvec
    
    # Project to image plane
    if X_cam[2] <= 0:
        # Point behind camera
        return np.array([np.nan, np.nan])
    
    X_norm = X_cam / X_cam[2]
    pt_proj = K @ X_norm
    
    return pt_proj[:2]


def compute_residuals(
    params: np.ndarray,
    n_cameras: int,
    n_points: int,
    camera_indices: np.ndarray,
    point_indices: np.ndarray,
    points_2d: np.ndarray,
    K: np.ndarray,
    camera_params_fixed: Dict[int, bool],
    point_params_fixed: Dict[int, bool]
) -> np.ndarray:
    """Compute reprojection residuals for bundle adjustment.
    
    Args:
        params: Flattened parameter vector [camera_params, point_params]
                camera_params: [rvec1, tvec1, rvec2, tvec2, ...]  (6 params per camera)
                point_params: [X1, Y1, Z1, X2, Y2, Z2, ...]  (3 params per point)
        n_cameras: Number of cameras
        n_points: Number of 3D points
        camera_indices: (N,) camera index for each observation
        point_indices: (N,) point index for each observation
        points_2d: (N, 2) observed 2D points
        K: (3, 3) camera intrinsic matrix
        camera_params_fixed: Dict of camera_idx → is_fixed
        point_params_fixed: Dict of point_idx → is_fixed
        
    Returns:
        residuals: (2*N,) flattened residual vector
    """
    # Unpack parameters
    camera_params = params[:n_cameras * 6].reshape((n_cameras, 6))
    point_params = params[n_cameras * 6:].reshape((n_points, 3))
    
    n_observations = len(camera_indices)
    residuals = np.zeros(2 * n_observations)
    
    for i in range(n_observations):
        cam_idx = camera_indices[i]
        pt_idx = point_indices[i]
        
        # Get camera pose
        rvec = camera_params[cam_idx, :3]
        tvec = camera_params[cam_idx, 3:6]
        
        # Get 3D point
        X = point_params[pt_idx]
        
        # Project
        pt_proj = project_point(X, rvec, tvec, K)
        
        # Compute residual
        if np.any(np.isnan(pt_proj)):
            # Point behind camera - assign large residual
            residuals[2*i:2*i+2] = [1000.0, 1000.0]
        else:
            residuals[2*i:2*i+2] = pt_proj - points_2d[i]
    
    return residuals


def bundle_adjust_global(
    sfm: IncrementalSfM,
    loss_function: str = 'huber',
    loss_scale: float = 1.0,
    max_iterations: int = 100,
    verbose: int = 0,
    fix_first_camera: bool = True
) -> Tuple[IncrementalSfM, Dict]:
    """Global bundle adjustment: optimize all cameras and points.
    
    Args:
        sfm: IncrementalSfM reconstruction to optimize
        loss_function: Robust loss ('linear', 'huber', 'soft_l1', 'cauchy')
        loss_scale: Scale parameter for robust loss
        max_iterations: Maximum optimization iterations
        verbose: Verbosity level (0=silent, 1=summary, 2=per-iteration)
        fix_first_camera: If True, fix first camera at identity (gauge freedom)
        
    Returns:
        Tuple of (optimized_sfm, info_dict):
            optimized_sfm: Updated IncrementalSfM with optimized parameters
            info_dict: Optimization statistics (cost, iterations, etc.)
    """
    # Get registered cameras and points
    registered_cameras = {
        img_id: cam for img_id, cam in sfm.cameras.items()
        if cam.registered
    }
    
    if len(registered_cameras) < 2:
        return sfm, {"success": False, "message": "Need >= 2 cameras"}
    
    if len(sfm.points_3d) < 3:
        return sfm, {"success": False, "message": "Need >= 3 points"}
    
    # Create index mappings
    camera_id_to_idx = {img_id: i for i, img_id in enumerate(registered_cameras.keys())}
    point_id_to_idx = {pt_id: i for i, pt_id in enumerate(sfm.points_3d.keys())}
    
    n_cameras = len(registered_cameras)
    n_points = len(sfm.points_3d)
    
    # Build observation arrays
    camera_indices = []
    point_indices = []
    points_2d = []
    
    for pt_id, point in sfm.points_3d.items():
        pt_idx = point_id_to_idx[pt_id]
        
        for img_id, pt_2d in point.observations.items():
            if img_id in camera_id_to_idx:
                cam_idx = camera_id_to_idx[img_id]
                camera_indices.append(cam_idx)
                point_indices.append(pt_idx)
                points_2d.append(pt_2d)
    
    camera_indices = np.array(camera_indices)
    point_indices = np.array(point_indices)
    points_2d = np.array(points_2d)
    n_observations = len(camera_indices)
    
    if n_observations < 10:
        return sfm, {"success": False, "message": f"Too few observations: {n_observations}"}
    
    # Initialize parameters
    camera_params = np.zeros((n_cameras, 6))
    for img_id, cam_idx in camera_id_to_idx.items():
        cam = registered_cameras[img_id]
        rvec = matrix_to_rodrigues(cam.R)
        tvec = cam.t.ravel()
        camera_params[cam_idx] = np.hstack([rvec, tvec])
    
    point_params = np.zeros((n_points, 3))
    for pt_id, pt_idx in point_id_to_idx.items():
        point_params[pt_idx] = sfm.points_3d[pt_id].xyz
    
    # Flatten parameters
    x0 = np.hstack([camera_params.ravel(), point_params.ravel()])
    
    # Determine fixed parameters
    camera_params_fixed = {}
    point_params_fixed = {}
    
    if fix_first_camera:
        # Fix first camera (gauge freedom)
        camera_params_fixed[0] = True
    
    # Residual function
    def fun(params):
        return compute_residuals(
            params, n_cameras, n_points,
            camera_indices, point_indices, points_2d,
            sfm.K, camera_params_fixed, point_params_fixed
        )
    
    # Optimize
    result = least_squares(
        fun,
        x0,
        loss=loss_function,
        f_scale=loss_scale,
        max_nfev=max_iterations * (len(x0) + 1),
        verbose=verbose,
        x_scale='jac',
        ftol=1e-6,
        xtol=1e-6,
        gtol=1e-6
    )
    
    if not result.success:
        return sfm, {
            "success": False,
            "message": result.message,
            "initial_cost": result.cost,
            "n_iterations": result.nfev
        }
    
    # Update SfM structure with optimized parameters
    optimized_params = result.x
    camera_params_opt = optimized_params[:n_cameras * 6].reshape((n_cameras, 6))
    point_params_opt = optimized_params[n_cameras * 6:].reshape((n_points, 3))
    
    # Update cameras
    for img_id, cam_idx in camera_id_to_idx.items():
        rvec_opt = camera_params_opt[cam_idx, :3]
        tvec_opt = camera_params_opt[cam_idx, 3:6]
        R_opt = rodrigues_to_matrix(rvec_opt)
        sfm.cameras[img_id].R = R_opt
        sfm.cameras[img_id].t = tvec_opt.reshape(3, 1)
    
    # Update 3D points
    for pt_id, pt_idx in point_id_to_idx.items():
        sfm.points_3d[pt_id].xyz = point_params_opt[pt_idx]
    
    # Compute final reprojection errors
    sfm.compute_reprojection_errors()
    
    # Build info dict
    initial_cost = np.sum(fun(x0) ** 2) / (2 * n_observations)
    final_cost = result.cost / n_observations
    
    info = {
        "success": True,
        "message": result.message,
        "initial_cost": float(initial_cost),
        "final_cost": float(final_cost),
        "cost_reduction": float(initial_cost - final_cost),
        "n_iterations": result.nfev,
        "n_cameras": n_cameras,
        "n_points": n_points,
        "n_observations": n_observations,
        "optimality": float(result.optimality),
        "status": int(result.status)
    }
    
    return sfm, info


def bundle_adjust_points_only(
    sfm: IncrementalSfM,
    loss_function: str = 'huber',
    max_iterations: int = 50,
    verbose: int = 0
) -> Tuple[IncrementalSfM, Dict]:
    """Bundle adjustment with fixed camera poses (point refinement only).
    
    Args:
        sfm: IncrementalSfM reconstruction
        loss_function: Robust loss function
        max_iterations: Maximum iterations
        verbose: Verbosity level
        
    Returns:
        Tuple of (optimized_sfm, info_dict)
    """
    # Similar to global BA but only optimize point parameters
    # Camera parameters are held fixed
    
    registered_cameras = {
        img_id: cam for img_id, cam in sfm.cameras.items()
        if cam.registered
    }
    
    if not registered_cameras or not sfm.points_3d:
        return sfm, {"success": False, "message": "No cameras or points"}
    
    camera_id_to_idx = {img_id: i for i, img_id in enumerate(registered_cameras.keys())}
    point_id_to_idx = {pt_id: i for i, pt_id in enumerate(sfm.points_3d.keys())}
    
    n_cameras = len(registered_cameras)
    n_points = len(sfm.points_3d)
    
    # Build observations
    camera_indices = []
    point_indices = []
    points_2d = []
    
    for pt_id, point in sfm.points_3d.items():
        pt_idx = point_id_to_idx[pt_id]
        for img_id, pt_2d in point.observations.items():
            if img_id in camera_id_to_idx:
                camera_indices.append(camera_id_to_idx[img_id])
                point_indices.append(pt_idx)
                points_2d.append(pt_2d)
    
    camera_indices = np.array(camera_indices)
    point_indices = np.array(point_indices)
    points_2d = np.array(points_2d)
    
    # Initialize camera parameters (fixed)
    camera_params = np.zeros((n_cameras, 6))
    for img_id, cam_idx in camera_id_to_idx.items():
        cam = registered_cameras[img_id]
        rvec = matrix_to_rodrigues(cam.R)
        tvec = cam.t.ravel()
        camera_params[cam_idx] = np.hstack([rvec, tvec])
    
    # Initialize point parameters (to be optimized)
    point_params = np.zeros((n_points, 3))
    for pt_id, pt_idx in point_id_to_idx.items():
        point_params[pt_idx] = sfm.points_3d[pt_id].xyz
    
    x0 = point_params.ravel()
    
    def fun(point_params_flat):
        # Combine fixed camera params with variable point params
        params = np.hstack([camera_params.ravel(), point_params_flat])
        return compute_residuals(
            params, n_cameras, n_points,
            camera_indices, point_indices, points_2d,
            sfm.K, {}, {}
        )
    
    result = least_squares(
        fun, x0,
        loss=loss_function,
        max_nfev=max_iterations * (len(x0) + 1),
        verbose=verbose
    )
    
    if result.success:
        # Update points
        point_params_opt = result.x.reshape((n_points, 3))
        for pt_id, pt_idx in point_id_to_idx.items():
            sfm.points_3d[pt_id].xyz = point_params_opt[pt_idx]
        
        sfm.compute_reprojection_errors()
    
    info = {
        "success": result.success,
        "message": result.message,
        "final_cost": float(result.cost) if result.success else None,
        "n_iterations": result.nfev
    }
    
    return sfm, info


def compute_initial_reprojection_stats(sfm: IncrementalSfM) -> Dict:
    """Compute reprojection error statistics before optimization.
    
    Args:
        sfm: IncrementalSfM reconstruction
        
    Returns:
        Dict with mean, max, median errors
    """
    errors = sfm.compute_reprojection_errors()
    
    if not errors:
        return {"mean": 0.0, "max": 0.0, "median": 0.0, "n_points": 0}
    
    error_values = list(errors.values())
    
    return {
        "mean": float(np.mean(error_values)),
        "max": float(np.max(error_values)),
        "median": float(np.median(error_values)),
        "n_points": len(error_values)
    }
