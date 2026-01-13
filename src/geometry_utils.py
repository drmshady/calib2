"""
Geometric utility functions for multi-view photogrammetry.

Provides core geometric operations for Structure-from-Motion (SfM) pipeline:
triangulation, reprojection error computation, cheirality checks, collinearity
validation, and rotation matrix utilities.

Functions:
    triangulate_dlt: Triangulate 3D point from multiple views using DLT
    triangulate_opencv: Triangulate using OpenCV's optimized method
    compute_reprojection_error: Calculate pixel reprojection errors
    check_cheirality: Verify points are in front of cameras
    check_collinearity: Validate non-collinearity of point sets
    validate_rotation_matrix: Check if matrix is valid SO(3)
    compute_ray_angle: Calculate angle between viewing rays
    fit_line_pca: Fit line to 3D points using PCA
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional, Dict
from scipy.linalg import svd


def validate_rotation_matrix(R: np.ndarray, tol: float = 1e-6) -> Tuple[bool, str]:
    """Validate if matrix is a valid rotation matrix (SO(3)).
    
    Checks:
        1. det(R) = +1 (proper rotation, not reflection)
        2. R^T @ R = I (orthonormality)
    
    Args:
        R: 3x3 matrix to validate
        tol: Tolerance for numerical checks
        
    Returns:
        Tuple of (is_valid, error_message):
            is_valid: True if valid rotation matrix
            error_message: Description of error, or empty string if valid
    """
    if R.shape != (3, 3):
        return False, f"Shape must be 3x3, got {R.shape}"
    
    # Check determinant
    det = np.linalg.det(R)
    if abs(det - 1.0) > tol:
        return False, f"det(R) = {det:.9f}, expected 1.0 (±{tol})"
    
    # Check orthonormality: R^T @ R should be identity
    identity_check = R.T @ R
    error = np.max(np.abs(identity_check - np.eye(3)))
    if error > tol:
        return False, f"R^T @ R error = {error:.9f}, expected < {tol}"
    
    return True, ""


def triangulate_dlt(
    points_2d: np.ndarray,
    projection_matrices: np.ndarray,
    normalize: bool = True
) -> np.ndarray:
    """Triangulate 3D point from multiple views using Direct Linear Transform (DLT).
    
    Solves for X in: P @ X = x (homogeneous coordinates)
    by constructing A @ X = 0 and solving via SVD.
    
    Args:
        points_2d: (N, 2) array of 2D points in undistorted pixel coordinates (one per view)
        projection_matrices: (N, 3, 4) array of projection matrices P = K @ [R | t]
        normalize: If True, normalize points for numerical stability
        
    Returns:
        point_3d: (3,) array in 3D (non-homogeneous)
        
    Raises:
        ValueError: If insufficient views (need >= 2)
    """
    n_views = len(points_2d)
    if n_views < 2:
        raise ValueError(f"Need >= 2 views for triangulation, got {n_views}")
    
    # Build design matrix A (2*N rows, 4 cols)
    A = np.zeros((2 * n_views, 4))
    
    for i, (pt, P) in enumerate(zip(points_2d, projection_matrices)):
        x, y = pt
        A[2*i] = x * P[2] - P[0]
        A[2*i + 1] = y * P[2] - P[1]
    
    # Solve A @ X = 0 using SVD: X is last column of V (smallest singular value)
    _, _, Vh = svd(A)
    X_homog = Vh[-1]
    
    # Convert to non-homogeneous coordinates
    X = X_homog[:3] / X_homog[3]
    
    return X


def triangulate_opencv(
    points_2d: List[np.ndarray],
    projection_matrices: List[np.ndarray]
) -> np.ndarray:
    """Triangulate 3D point using OpenCV's optimized two-view method.
    
    Uses cv2.triangulatePoints for the first two views, then optionally
    refines with additional views via DLT.
    
    Args:
        points_2d: List of (2,) 2D points in undistorted pixel coordinates
        projection_matrices: List of (3, 4) projection matrices P = K @ [R | t]
        
    Returns:
        point_3d: (3,) array in 3D
    """
    if len(points_2d) < 2:
        raise ValueError("Need >= 2 views for triangulation")
    
    # Use first two views with OpenCV
    pt1 = points_2d[0].reshape(2, 1)
    pt2 = points_2d[1].reshape(2, 1)
    P1 = projection_matrices[0]
    P2 = projection_matrices[1]
    
    # cv2.triangulatePoints returns homogeneous coordinates (4, N)
    X_homog = cv2.triangulatePoints(P1, P2, pt1, pt2)
    X = X_homog[:3, 0] / X_homog[3, 0]
    
    # If more views available, refine with DLT
    if len(points_2d) > 2:
        X = triangulate_dlt(
            np.array(points_2d),
            np.array(projection_matrices)
        )
    
    return X


def compute_reprojection_error(
    points_3d: np.ndarray,
    points_2d: np.ndarray,
    K: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    return_per_point: bool = False
) -> np.ndarray:
    """Compute reprojection error for 3D points in a camera view.
    
    Projects 3D points to image plane and computes pixel distance to observations.
    
    Args:
        points_3d: (N, 3) array of 3D points in world coordinates (mm)
        points_2d: (N, 2) array of observed 2D points in undistorted pixel coordinates
        K: (3, 3) camera intrinsic matrix
        R: (3, 3) camera rotation matrix (world → camera)
        t: (3, 1) or (3,) camera translation vector
        return_per_point: If True, return per-point errors; else return RMSE
        
    Returns:
        If return_per_point=True: (N,) array of pixel errors
        If return_per_point=False: scalar RMSE in pixels
    """
    t = np.array(t).reshape(3, 1)
    
    # Transform to camera frame: X_cam = R @ X_world + t
    points_3d = np.array(points_3d)
    if points_3d.ndim == 1:
        points_3d = points_3d.reshape(1, 3)
    
    X_cam = (R @ points_3d.T + t).T  # (N, 3)
    
    # Project to image plane: x = K @ X_cam, then normalize by Z
    X_proj = (K @ X_cam.T).T  # (N, 3) homogeneous
    points_proj = X_proj[:, :2] / X_proj[:, 2:3]  # (N, 2) in pixels
    
    # Compute errors
    points_2d = np.array(points_2d)
    if points_2d.ndim == 1:
        points_2d = points_2d.reshape(1, 2)
    
    errors = np.linalg.norm(points_proj - points_2d, axis=1)  # (N,)
    
    if return_per_point:
        return errors
    else:
        return np.sqrt(np.mean(errors ** 2))  # RMSE


def check_cheirality(
    points_3d: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    min_depth: float = 0.0
) -> np.ndarray:
    """Check if 3D points are in front of camera (positive depth).
    
    Cheirality constraint: points must have Z > 0 in camera frame.
    
    Args:
        points_3d: (N, 3) array of 3D points in world coordinates
        R: (3, 3) camera rotation matrix (world → camera)
        t: (3, 1) or (3,) camera translation vector
        min_depth: Minimum required depth in mm (default 0.0)
        
    Returns:
        valid_mask: (N,) boolean array, True if point is in front of camera
    """
    t = np.array(t).reshape(3, 1)
    points_3d = np.array(points_3d)
    if points_3d.ndim == 1:
        points_3d = points_3d.reshape(1, 3)
    
    # Transform to camera frame
    X_cam = (R @ points_3d.T + t).T  # (N, 3)
    
    # Check Z coordinate (depth)
    depths = X_cam[:, 2]
    valid_mask = depths > min_depth
    
    return valid_mask


def compute_ray_angle(
    point_3d: np.ndarray,
    camera_center_1: np.ndarray,
    camera_center_2: np.ndarray
) -> float:
    """Compute angle between two viewing rays to a 3D point.
    
    Used for DOP (Dilution of Precision) checks: angle should be >= 5° for
    reliable triangulation.
    
    Args:
        point_3d: (3,) 3D point position
        camera_center_1: (3,) first camera center position
        camera_center_2: (3,) second camera center position
        
    Returns:
        angle_deg: Angle between rays in degrees [0, 180]
    """
    # Ray vectors from cameras to point
    ray1 = point_3d - camera_center_1
    ray2 = point_3d - camera_center_2
    
    # Normalize
    ray1 = ray1 / np.linalg.norm(ray1)
    ray2 = ray2 / np.linalg.norm(ray2)
    
    # Compute angle: cos(θ) = ray1 · ray2
    cos_angle = np.clip(np.dot(ray1, ray2), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg


def check_collinearity(
    points_3d: np.ndarray,
    threshold_mm: float = 5.0
) -> Tuple[bool, float, np.ndarray]:
    """Check if 3D points are collinear (anti-hinge check).
    
    Fits line to points using PCA and computes maximum perpendicular distance.
    Points are considered non-collinear if max distance > threshold.
    
    Args:
        points_3d: (N, 3) array of 3D points
        threshold_mm: Minimum distance for non-collinearity (default 5.0mm)
        
    Returns:
        Tuple of (is_non_collinear, max_distance, line_direction):
            is_non_collinear: True if points are sufficiently non-collinear
            max_distance: Maximum perpendicular distance to fitted line (mm)
            line_direction: (3,) unit vector along line direction
    """
    points_3d = np.array(points_3d)
    if len(points_3d) < 3:
        # Need >= 3 points to check collinearity
        return True, np.inf, np.zeros(3)
    
    # Fit line using PCA
    centroid = np.mean(points_3d, axis=0)
    centered = points_3d - centroid
    
    # Compute covariance and get principal direction
    cov_matrix = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    
    # Principal direction (largest eigenvalue)
    line_direction = eigenvectors[:, -1]
    
    # Compute perpendicular distances to line
    distances = []
    for pt in points_3d:
        # Vector from centroid to point
        v = pt - centroid
        # Project onto line direction
        proj_length = np.dot(v, line_direction)
        proj = proj_length * line_direction
        # Perpendicular component
        perp = v - proj
        distance = np.linalg.norm(perp)
        distances.append(distance)
    
    max_distance = max(distances)
    is_non_collinear = max_distance > threshold_mm
    
    return is_non_collinear, max_distance, line_direction


def fit_line_pca(points_3d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Fit line to 3D points using Principal Component Analysis.
    
    Args:
        points_3d: (N, 3) array of 3D points
        
    Returns:
        Tuple of (centroid, direction):
            centroid: (3,) line center point
            direction: (3,) unit vector along line direction
    """
    points_3d = np.array(points_3d)
    
    # Compute centroid
    centroid = np.mean(points_3d, axis=0)
    
    # Center points
    centered = points_3d - centroid
    
    # PCA via SVD
    _, _, Vh = svd(centered, full_matrices=False)
    direction = Vh[0]  # First principal component
    
    return centroid, direction


def check_triangle_area(
    points_3d: np.ndarray,
    threshold_mm2: float = 10.0
) -> Tuple[bool, float]:
    """Check if 3 points form a triangle with sufficient area (anti-hinge check).
    
    Used to validate that points are not collinear by ensuring they form
    a triangle with area >= threshold.
    
    Args:
        points_3d: (3, 3) array of 3 points
        threshold_mm2: Minimum area in mm² (default 10.0)
        
    Returns:
        Tuple of (is_valid, area):
            is_valid: True if area >= threshold
            area: Triangle area in mm²
    """
    if len(points_3d) != 3:
        raise ValueError(f"Need exactly 3 points for triangle, got {len(points_3d)}")
    
    # Triangle area using cross product: A = 0.5 * ||(p2-p1) × (p3-p1)||
    v1 = points_3d[1] - points_3d[0]
    v2 = points_3d[2] - points_3d[0]
    cross = np.cross(v1, v2)
    area = 0.5 * np.linalg.norm(cross)
    
    is_valid = area >= threshold_mm2
    
    return is_valid, area


def compute_camera_center(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Compute camera center in world coordinates.
    
    Camera center C: X_world = -R^T @ t
    (OpenCV convention: X_cam = R @ X_world + t)
    
    Args:
        R: (3, 3) rotation matrix (world → camera)
        t: (3, 1) or (3,) translation vector
        
    Returns:
        camera_center: (3,) position in world coordinates
    """
    t = np.array(t).reshape(3, 1)
    camera_center = -R.T @ t
    return camera_center.ravel()


def essential_matrix_to_pose(
    E: np.ndarray,
    points1_norm: np.ndarray,
    points2_norm: np.ndarray,
    K: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Recover camera pose from essential matrix with cheirality check.
    
    Decomposes essential matrix into 4 possible (R, t) solutions and selects
    the one with most points in front of both cameras.
    
    Args:
        E: (3, 3) essential matrix
        points1_norm: (N, 2) normalized points in first image
        points2_norm: (N, 2) normalized points in second image
        K: (3, 3) camera intrinsic matrix (for projection matrices)
        
    Returns:
        Tuple of (R, t, mask):
            R: (3, 3) rotation matrix (camera1 → camera2)
            t: (3, 1) translation vector (unit norm)
            mask: (N,) boolean array of inlier points
    """
    # Decompose essential matrix: E → R1, R2, t
    # OpenCV returns 4 solutions: (R1, t), (R1, -t), (R2, t), (R2, -t)
    R1, R2, t_unit = cv2.decomposeEssentialMat(E)
    
    # Camera 1 at origin: P1 = K @ [I | 0]
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    
    # Test all 4 possible solutions
    solutions = [
        (R1, t_unit),
        (R1, -t_unit),
        (R2, t_unit),
        (R2, -t_unit)
    ]
    
    best_solution = None
    max_good_points = 0
    best_mask = None
    
    for R, t in solutions:
        # P2 = K @ [R | t]
        P2 = K @ np.hstack([R, t])
        
        # Triangulate points
        good_count = 0
        mask = np.zeros(len(points1_norm), dtype=bool)
        
        for i, (pt1, pt2) in enumerate(zip(points1_norm, points2_norm)):
            # Triangulate
            pt1_h = pt1.reshape(2, 1)
            pt2_h = pt2.reshape(2, 1)
            X_h = cv2.triangulatePoints(P1, P2, pt1_h, pt2_h)
            X = X_h[:3, 0] / X_h[3, 0]
            
            # Check cheirality in both cameras
            # Camera 1: Z > 0 (X is in world = camera 1 frame)
            Z1 = X[2]
            
            # Camera 2: X_cam2 = R @ X + t
            X_cam2 = R @ X + t.ravel()
            Z2 = X_cam2[2]
            
            if Z1 > 0 and Z2 > 0:
                good_count += 1
                mask[i] = True
        
        # Select solution with most points in front of both cameras
        if good_count > max_good_points:
            max_good_points = good_count
            best_solution = (R, t)
            best_mask = mask
    
    if best_solution is None or max_good_points == 0:
        raise ValueError("No valid pose found: all points behind cameras")
    
    R, t = best_solution
    
    return R, t, best_mask
