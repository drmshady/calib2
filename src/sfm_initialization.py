"""
Two-view Structure-from-Motion initialization.

Implements initial camera pose estimation and 3D point triangulation from
two views using Essential matrix decomposition with RANSAC. Enforces DOP
(Dilution of Precision) and anti-hinge quality checks.

Class:
    SfMInitializer: Two-view SfM with essential matrix + pose recovery

Quality Gates:
    - DOP Check: Mean ray angle >= 5° (prevents poor triangulation geometry)
    - Anti-Hinge Check: Triangle area >= 10mm² (prevents collinear points)
    - Cheirality: All points must be in front of both cameras
"""

import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

import geometry_utils
from geometry_utils import (
    validate_rotation_matrix,
    triangulate_opencv,
    compute_reprojection_error,
    check_cheirality,
    check_triangle_area,
    compute_ray_angle,
    compute_camera_center,
    essential_matrix_to_pose
)


@dataclass
class SfMInitResult:
    """Result of two-view SfM initialization.
    
    Attributes:
        success: True if initialization succeeded
        R: (3, 3) rotation matrix (camera1 → camera2)
        t: (3, 1) translation vector (unit norm, scale ambiguity)
        points_3d: (N, 3) triangulated 3D points in camera1 frame
        inlier_mask: (N,) boolean array of inlier points
        mean_ray_angle: Mean angle between viewing rays (degrees)
        reprojection_error: Mean reprojection error (pixels)
        error_message: Error description if failed
    """
    success: bool
    R: Optional[np.ndarray] = None
    t: Optional[np.ndarray] = None
    points_3d: Optional[np.ndarray] = None
    inlier_mask: Optional[np.ndarray] = None
    mean_ray_angle: float = 0.0
    reprojection_error: float = 0.0
    error_message: str = ""


class SfMInitializer:
    """Two-view Structure-from-Motion initializer.
    
    Estimates camera pose and triangulates 3D points from two views using:
    1. Essential matrix estimation (Nister's 5-point + RANSAC)
    2. Pose recovery with cheirality check
    3. Initial triangulation
    4. DOP validation (ray angle >= 5°)
    5. Anti-hinge validation (triangle area >= 10mm²)
    
    Attributes:
        ransac_threshold: RANSAC threshold in pixels (default 1.0)
        ransac_confidence: RANSAC confidence level (default 0.999)
        min_ray_angle_deg: Minimum mean ray angle for DOP check (default 5.0)
        min_triangle_area_mm2: Minimum triangle area for anti-hinge (default 10.0)
        min_inlier_ratio: Minimum ratio of inliers (default 0.5)
    """
    
    def __init__(
        self,
        ransac_threshold: float = 1.0,
        ransac_confidence: float = 0.999,
        min_ray_angle_deg: float = 5.0,
        min_triangle_area_mm2: float = 10.0,
        min_inlier_ratio: float = 0.5
    ):
        """Initialize SfM initializer with quality thresholds.
        
        Args:
            ransac_threshold: RANSAC threshold in pixels
            ransac_confidence: RANSAC confidence [0, 1]
            min_ray_angle_deg: Minimum mean ray angle (DOP check)
            min_triangle_area_mm2: Minimum triangle area (anti-hinge)
            min_inlier_ratio: Minimum inlier ratio [0, 1]
        """
        self.ransac_threshold = ransac_threshold
        self.ransac_confidence = ransac_confidence
        self.min_ray_angle_deg = min_ray_angle_deg
        self.min_triangle_area_mm2 = min_triangle_area_mm2
        self.min_inlier_ratio = min_inlier_ratio
    
    def initialize(
        self,
        points1_norm: np.ndarray,
        points2_norm: np.ndarray,
        K: np.ndarray
    ) -> SfMInitResult:
        """Initialize SfM from two views.
        
        Args:
            points1_norm: (N, 2) normalized points in first image
            points2_norm: (N, 2) normalized points in second image
            K: (3, 3) camera intrinsic matrix
            
        Returns:
            SfMInitResult with pose, 3D points, and quality metrics
        """
        n_points = len(points1_norm)
        
        if n_points < 5:
            return SfMInitResult(
                success=False,
                error_message=f"Need >= 5 correspondences, got {n_points}"
            )
        
        # Step 1: Estimate essential matrix using RANSAC (Nister's 5-point)
        E, mask_essential = cv2.findEssentialMat(
            points1_norm,
            points2_norm,
            focal=1.0,  # Normalized coordinates (K already applied)
            pp=(0.0, 0.0),
            method=cv2.RANSAC,
            prob=self.ransac_confidence,
            threshold=self.ransac_threshold
        )
        
        if E is None:
            return SfMInitResult(
                success=False,
                error_message="Essential matrix estimation failed"
            )
        
        mask_essential = mask_essential.ravel().astype(bool)
        n_inliers = np.sum(mask_essential)
        inlier_ratio = n_inliers / n_points
        
        if inlier_ratio < self.min_inlier_ratio:
            return SfMInitResult(
                success=False,
                error_message=f"Insufficient inliers: {inlier_ratio:.2%} < {self.min_inlier_ratio:.2%}"
            )
        
        # Step 2: Recover pose with cheirality check
        try:
            R, t, mask_cheirality = essential_matrix_to_pose(
                E,
                points1_norm[mask_essential],
                points2_norm[mask_essential],
                K
            )
        except ValueError as e:
            return SfMInitResult(
                success=False,
                error_message=f"Pose recovery failed: {e}"
            )
        
        # Combine masks: essential RANSAC inliers AND cheirality inliers
        combined_mask = np.zeros(n_points, dtype=bool)
        essential_indices = np.where(mask_essential)[0]
        cheirality_indices = essential_indices[mask_cheirality]
        combined_mask[cheirality_indices] = True
        
        n_final_inliers = np.sum(combined_mask)
        
        if n_final_inliers < 5:
            return SfMInitResult(
                success=False,
                error_message=f"Too few points after cheirality: {n_final_inliers}"
            )
        
        # Step 3: Triangulate all inlier points
        points1_inliers = points1_norm[combined_mask]
        points2_inliers = points2_norm[combined_mask]
        
        # Projection matrices: P1 = K @ [I | 0], P2 = K @ [R | t]
        P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
        P2 = K @ np.hstack([R, t])
        
        points_3d = []
        for pt1, pt2 in zip(points1_inliers, points2_inliers):
            try:
                X = triangulate_opencv([pt1, pt2], [P1, P2])
                points_3d.append(X)
            except Exception:
                continue
        
        if len(points_3d) < 3:
            return SfMInitResult(
                success=False,
                error_message=f"Triangulation failed: only {len(points_3d)} points"
            )
        
        points_3d = np.array(points_3d)
        
        # Step 4: DOP Check - Mean ray angle >= 5°
        camera_center1 = np.array([0, 0, 0])  # Camera 1 at origin
        camera_center2 = compute_camera_center(R, t)
        
        ray_angles = []
        for X in points_3d:
            angle = compute_ray_angle(X, camera_center1, camera_center2)
            ray_angles.append(angle)
        
        mean_ray_angle = np.mean(ray_angles)
        
        if mean_ray_angle < self.min_ray_angle_deg:
            return SfMInitResult(
                success=False,
                error_message=f"DOP check failed: mean ray angle {mean_ray_angle:.2f}° < {self.min_ray_angle_deg}°"
            )
        
        # Step 5: Anti-Hinge Check - Select 3 points and check triangle area
        if len(points_3d) >= 3:
            # Check first 3 points (or random sample)
            sample_indices = np.random.choice(len(points_3d), size=3, replace=False)
            sample_points = points_3d[sample_indices]
            
            is_valid, area = check_triangle_area(sample_points, self.min_triangle_area_mm2)
            
            if not is_valid:
                return SfMInitResult(
                    success=False,
                    error_message=f"Anti-hinge check failed: triangle area {area:.2f}mm² < {self.min_triangle_area_mm2}mm²"
                )
        
        # Step 6: Compute reprojection error
        reproj_error1 = compute_reprojection_error(
            points_3d, points1_inliers, K,
            np.eye(3), np.zeros((3, 1))
        )
        reproj_error2 = compute_reprojection_error(
            points_3d, points2_inliers, K, R, t
        )
        mean_reproj_error = (reproj_error1 + reproj_error2) / 2
        
        # Success!
        return SfMInitResult(
            success=True,
            R=R,
            t=t,
            points_3d=points_3d,
            inlier_mask=combined_mask,
            mean_ray_angle=mean_ray_angle,
            reprojection_error=mean_reproj_error,
            error_message=""
        )
    
    def initialize_with_known_scale(
        self,
        points1_norm: np.ndarray,
        points2_norm: np.ndarray,
        K: np.ndarray,
        known_distance_3d: float,
        point_idx1: int,
        point_idx2: int
    ) -> SfMInitResult:
        """Initialize SfM with known scale from a reference distance.
        
        Resolves scale ambiguity by constraining distance between two 3D points.
        
        Args:
            points1_norm: (N, 2) normalized points in first image
            points2_norm: (N, 2) normalized points in second image
            K: (3, 3) camera intrinsic matrix
            known_distance_3d: Known distance between two points (mm)
            point_idx1: Index of first reference point
            point_idx2: Index of second reference point
            
        Returns:
            SfMInitResult with scaled pose and 3D points
        """
        # First run standard initialization
        result = self.initialize(points1_norm, points2_norm, K)
        
        if not result.success:
            return result
        
        # Get the two reference points
        inlier_indices = np.where(result.inlier_mask)[0]
        
        if point_idx1 not in inlier_indices or point_idx2 not in inlier_indices:
            return SfMInitResult(
                success=False,
                error_message="Reference points are not inliers"
            )
        
        # Find positions in inlier array
        inlier_map = {idx: i for i, idx in enumerate(inlier_indices)}
        ref_idx1 = inlier_map[point_idx1]
        ref_idx2 = inlier_map[point_idx2]
        
        # Compute scale factor
        pt1_3d = result.points_3d[ref_idx1]
        pt2_3d = result.points_3d[ref_idx2]
        current_distance = np.linalg.norm(pt2_3d - pt1_3d)
        
        if current_distance < 1e-6:
            return SfMInitResult(
                success=False,
                error_message="Reference points are too close (degenerate)"
            )
        
        scale_factor = known_distance_3d / current_distance
        
        # Scale translation and 3D points
        t_scaled = result.t * scale_factor
        points_3d_scaled = result.points_3d * scale_factor
        
        # Recompute reprojection error with scaled geometry
        points1_inliers = points1_norm[result.inlier_mask]
        points2_inliers = points2_norm[result.inlier_mask]
        
        reproj_error1 = compute_reprojection_error(
            points_3d_scaled, points1_inliers, K,
            np.eye(3), np.zeros((3, 1))
        )
        reproj_error2 = compute_reprojection_error(
            points_3d_scaled, points2_inliers, K, result.R, t_scaled
        )
        mean_reproj_error = (reproj_error1 + reproj_error2) / 2
        
        return SfMInitResult(
            success=True,
            R=result.R,
            t=t_scaled,
            points_3d=points_3d_scaled,
            inlier_mask=result.inlier_mask,
            mean_ray_angle=result.mean_ray_angle,
            reprojection_error=mean_reproj_error,
            error_message=""
        )
