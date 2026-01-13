"""
Incremental Structure-from-Motion reconstruction engine.

Builds 3D structure by incrementally registering cameras via PnP and
triangulating new points. Maintains camera poses and 3D point cloud with
feature tracks.

**COORDINATE CONVENTION (CRITICAL):**
    All 2D observations MUST be undistorted pixels:
    - Distortion removed via: cv2.undistortPoints(src, K, D, P=K)
    - Output format: Pixel coordinates (e.g., 3124.5, 2045.8)
    - NOT normalized coordinates (use P=K, not default P=None)
    
    All 3D coordinates MUST be in millimeters (mm):
    - Scale established via known AprilTag geometry (7.0mm coded size)
    - Never use meters (explicit ban per PROJECT_CONSTITUTION.md)
    
    PnP registration expects:
    - 2D points: Undistorted pixels (distortion already removed)
    - 3D points: Millimeters in L-frame (world coordinates)
    - distCoeffs=None in cv2.solvePnPRansac (distortion pre-removed)

Class:
    IncrementalSfM: Incremental reconstruction with PnP + triangulation

Features:
    - Camera registration via PnP + RANSAC (undistorted pixels)
    - Incremental triangulation with ray angle validation
    - Next-view selection based on 2D-3D correspondences
    - Collinearity checks (bridge validation)
    - Structure export to JSON (L-frame, mm units)
"""

import numpy as np
import cv2
import json
from typing import Dict, List, Tuple, Optional, Set
from dataclasses import dataclass, field
from pathlib import Path

import geometry_utils
from geometry_utils import (
    triangulate_opencv,
    compute_reprojection_error,
    check_cheirality,
    check_collinearity,
    compute_ray_angle,
    compute_camera_center,
    validate_rotation_matrix
)


@dataclass
class Camera:
    """Camera pose in L-frame.
    
    Attributes:
        image_id: Unique image identifier
        R: (3, 3) rotation matrix (world → camera)
        t: (3, 1) translation vector (mm)
        K: (3, 3) intrinsic matrix
        registered: True if camera pose has been estimated
    """
    image_id: str
    R: np.ndarray
    t: np.ndarray
    K: np.ndarray
    registered: bool = False


@dataclass
class Point3D:
    """3D point in L-frame with feature track.
    
    Attributes:
        point_id: Unique point identifier
        xyz: (3,) 3D coordinates (mm)
        observations: Dict mapping image_id → (x, y) 2D coordinates
        color: Optional (R, G, B) color tuple
        error: Reprojection error (pixels)
    """
    point_id: int
    xyz: np.ndarray
    observations: Dict[str, np.ndarray] = field(default_factory=dict)
    color: Optional[Tuple[int, int, int]] = None
    error: float = 0.0


class IncrementalSfM:
    """Incremental Structure-from-Motion reconstruction engine.
    
    Attributes:
        cameras: Dict of image_id → Camera
        points_3d: Dict of point_id → Point3D
        feature_tracks: Dict of track_id → List[(image_id, feature_idx)]
        K: Camera intrinsic matrix (assumed constant)
        min_ray_angle_deg: Minimum ray angle for triangulation (default 5°)
        pnp_ransac_threshold: PnP RANSAC threshold in pixels (default 3.0)
        min_triangulation_views: Minimum views to triangulate (default 2)
    """
    
    def __init__(
        self,
        K: np.ndarray,
        min_ray_angle_deg: float = 5.0,
        pnp_ransac_threshold: float = 3.0,
        min_triangulation_views: int = 2
    ):
        """Initialize incremental SfM engine.
        
        Args:
            K: (3, 3) camera intrinsic matrix
            min_ray_angle_deg: Minimum ray angle for triangulation
            pnp_ransac_threshold: PnP RANSAC threshold (pixels)
            min_triangulation_views: Minimum views to triangulate point
        """
        self.cameras: Dict[str, Camera] = {}
        self.points_3d: Dict[int, Point3D] = {}
        self.feature_tracks: Dict[int, List[Tuple[str, int]]] = {}
        self.K = K
        self.min_ray_angle_deg = min_ray_angle_deg
        self.pnp_ransac_threshold = pnp_ransac_threshold
        self.min_triangulation_views = min_triangulation_views
        self._next_point_id = 0
    
    def add_initial_pair(
        self,
        image_id1: str,
        image_id2: str,
        R: np.ndarray,
        t: np.ndarray,
        points_3d: np.ndarray,
        points_2d_1: np.ndarray,
        points_2d_2: np.ndarray,
        inlier_mask: np.ndarray
    ):
        """Add initial camera pair and triangulated points.
        
        Args:
            image_id1: First image identifier
            image_id2: Second image identifier
            R: (3, 3) rotation (camera1 → camera2)
            t: (3, 1) translation
            points_3d: (N, 3) triangulated 3D points
            points_2d_1: (M, 2) all 2D points in image 1
            points_2d_2: (M, 2) all 2D points in image 2
            inlier_mask: (M,) boolean mask of triangulated points
        """
        # Add cameras
        self.cameras[image_id1] = Camera(
            image_id=image_id1,
            R=np.eye(3),
            t=np.zeros((3, 1)),
            K=self.K,
            registered=True
        )
        
        self.cameras[image_id2] = Camera(
            image_id=image_id2,
            R=R,
            t=t,
            K=self.K,
            registered=True
        )
        
        # Add 3D points and create feature tracks
        inlier_indices = np.where(inlier_mask)[0]
        
        for i, idx in enumerate(inlier_indices):
            point_id = self._next_point_id
            self._next_point_id += 1
            
            # Create 3D point
            self.points_3d[point_id] = Point3D(
                point_id=point_id,
                xyz=points_3d[i],
                observations={
                    image_id1: points_2d_1[idx],
                    image_id2: points_2d_2[idx]
                }
            )
            
            # Create feature track
            self.feature_tracks[point_id] = [
                (image_id1, idx),
                (image_id2, idx)
            ]
    
    def register_camera(
        self,
        image_id: str,
        points_2d: np.ndarray,
        correspondences: List[Tuple[int, int]]
    ) -> Tuple[bool, str]:
        """Register new camera via PnP + RANSAC.
        
        Args:
            image_id: Image identifier
            points_2d: (N, 2) all 2D points in this image
            correspondences: List of (point_3d_id, feature_idx) correspondences
            
        Returns:
            Tuple of (success, error_message)
        """
        if len(correspondences) < 4:
            return False, f"Need >= 4 correspondences, got {len(correspondences)}"
        
        # Build 3D-2D correspondences
        points_3d_list = []
        points_2d_list = []
        
        for point_id, feat_idx in correspondences:
            if point_id in self.points_3d:
                points_3d_list.append(self.points_3d[point_id].xyz)
                points_2d_list.append(points_2d[feat_idx])
        
        if len(points_3d_list) < 4:
            return False, "Insufficient valid 3D-2D correspondences"
        
        points_3d_arr = np.array(points_3d_list)
        points_2d_arr = np.array(points_2d_list)
        
        # Solve PnP with RANSAC
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            points_3d_arr,
            points_2d_arr,
            self.K,
            None,  # No distortion (already undistorted)
            iterationsCount=1000,
            reprojectionError=self.pnp_ransac_threshold,
            confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP
        )
        
        if not success:
            return False, "PnP RANSAC failed to find solution"
        
        # Convert rotation vector to matrix
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3, 1)
        
        # Validate rotation matrix
        is_valid, error_msg = validate_rotation_matrix(R)
        if not is_valid:
            return False, f"Invalid rotation matrix: {error_msg}"
        
        # Check inlier ratio
        inlier_ratio = len(inliers) / len(points_3d_list)
        if inlier_ratio < 0.3:
            return False, f"Low inlier ratio: {inlier_ratio:.2%}"
        
        # Add camera
        self.cameras[image_id] = Camera(
            image_id=image_id,
            R=R,
            t=t,
            K=self.K,
            registered=True
        )
        
        # Update observations for inlier points
        if inliers is not None:
            for idx in inliers.ravel():
                point_id, feat_idx = correspondences[idx]
                if point_id in self.points_3d:
                    self.points_3d[point_id].observations[image_id] = points_2d[feat_idx]
        
        return True, ""
    
    def triangulate_new_points(
        self,
        feature_tracks: Dict[int, List[Tuple[str, int]]],
        all_points_2d: Dict[str, np.ndarray]
    ) -> int:
        """Triangulate new 3D points from feature tracks.
        
        Args:
            feature_tracks: Dict of track_id → [(image_id, feature_idx), ...]
            all_points_2d: Dict of image_id → (N, 2) 2D points
            
        Returns:
            Number of new points triangulated
        """
        new_points_count = 0
        
        for track_id, track in feature_tracks.items():
            # Skip if already triangulated
            if track_id in self.points_3d:
                continue
            
            # Get registered cameras in this track
            registered_views = [
                (img_id, feat_idx)
                for img_id, feat_idx in track
                if img_id in self.cameras and self.cameras[img_id].registered
            ]
            
            if len(registered_views) < self.min_triangulation_views:
                continue
            
            # Build projection matrices and 2D points
            projection_matrices = []
            points_2d_list = []
            camera_centers = []
            
            for img_id, feat_idx in registered_views:
                cam = self.cameras[img_id]
                P = self.K @ np.hstack([cam.R, cam.t])
                projection_matrices.append(P)
                points_2d_list.append(all_points_2d[img_id][feat_idx])
                camera_centers.append(compute_camera_center(cam.R, cam.t))
            
            # Triangulate
            try:
                X = triangulate_opencv(points_2d_list, projection_matrices)
            except Exception:
                continue
            
            # Check cheirality in all views
            all_valid = True
            for img_id, _ in registered_views:
                cam = self.cameras[img_id]
                valid_mask = check_cheirality(X.reshape(1, 3), cam.R, cam.t)
                if not valid_mask[0]:
                    all_valid = False
                    break
            
            if not all_valid:
                continue
            
            # Check ray angle (DOP)
            if len(camera_centers) >= 2:
                angles = []
                for i in range(len(camera_centers) - 1):
                    angle = compute_ray_angle(X, camera_centers[i], camera_centers[i+1])
                    angles.append(angle)
                
                mean_angle = np.mean(angles)
                if mean_angle < self.min_ray_angle_deg:
                    continue
            
            # Add 3D point
            point_id = track_id if track_id >= self._next_point_id else self._next_point_id
            self._next_point_id = max(self._next_point_id, point_id + 1)
            
            observations = {
                img_id: all_points_2d[img_id][feat_idx]
                for img_id, feat_idx in registered_views
            }
            
            self.points_3d[point_id] = Point3D(
                point_id=point_id,
                xyz=X,
                observations=observations
            )
            
            self.feature_tracks[point_id] = registered_views
            new_points_count += 1
        
        return new_points_count
    
    def select_next_view(
        self,
        candidate_image_ids: List[str],
        all_correspondences: Dict[str, List[Tuple[int, int]]]
    ) -> Optional[str]:
        """Select next view to register based on 2D-3D correspondences.
        
        Args:
            candidate_image_ids: List of unregistered image IDs
            all_correspondences: Dict of image_id → [(point_3d_id, feature_idx), ...]
            
        Returns:
            Best image_id to register next, or None if no good candidate
        """
        best_image_id = None
        max_correspondences = 0
        
        for image_id in candidate_image_ids:
            if image_id in self.cameras and self.cameras[image_id].registered:
                continue
            
            # Count valid 3D-2D correspondences
            if image_id not in all_correspondences:
                continue
            
            valid_count = sum(
                1 for point_id, _ in all_correspondences[image_id]
                if point_id in self.points_3d
            )
            
            if valid_count > max_correspondences:
                max_correspondences = valid_count
                best_image_id = image_id
        
        return best_image_id if max_correspondences >= 4 else None
    
    def check_bridge_collinearity(
        self,
        point_ids: List[int],
        threshold_mm: float = 5.0
    ) -> Tuple[bool, float]:
        """Check collinearity of bridge points (anti-hinge enforcement).
        
        Args:
            point_ids: List of point IDs to check
            threshold_mm: Maximum distance for collinearity check
            
        Returns:
            Tuple of (is_non_collinear, max_distance)
        """
        if len(point_ids) < 3:
            return True, np.inf
        
        points_3d = np.array([
            self.points_3d[pid].xyz
            for pid in point_ids
            if pid in self.points_3d
        ])
        
        if len(points_3d) < 3:
            return True, np.inf
        
        is_non_collinear, max_distance, _ = check_collinearity(points_3d, threshold_mm)
        
        return is_non_collinear, max_distance
    
    def compute_reprojection_errors(self) -> Dict[int, float]:
        """Compute reprojection error for all 3D points.
        
        Returns:
            Dict of point_id → reprojection_error (pixels)
        """
        errors = {}
        
        for point_id, point in self.points_3d.items():
            point_errors = []
            
            for img_id, pt_2d in point.observations.items():
                if img_id not in self.cameras:
                    continue
                
                cam = self.cameras[img_id]
                if not cam.registered:
                    continue
                
                error = compute_reprojection_error(
                    point.xyz.reshape(1, 3),
                    pt_2d.reshape(1, 2),
                    cam.K,
                    cam.R,
                    cam.t,
                    return_per_point=True
                )[0]
                
                point_errors.append(error)
            
            if point_errors:
                errors[point_id] = np.mean(point_errors)
                point.error = errors[point_id]
        
        return errors
    
    def get_structure_summary(self) -> Dict:
        """Get summary statistics of reconstruction.
        
        Returns:
            Dict with cameras count, points count, track lengths, etc.
        """
        registered_cameras = sum(1 for cam in self.cameras.values() if cam.registered)
        
        track_lengths = [
            len(point.observations)
            for point in self.points_3d.values()
        ]
        
        return {
            "n_cameras_total": len(self.cameras),
            "n_cameras_registered": registered_cameras,
            "n_points_3d": len(self.points_3d),
            "mean_track_length": np.mean(track_lengths) if track_lengths else 0,
            "median_track_length": np.median(track_lengths) if track_lengths else 0,
            "min_track_length": min(track_lengths) if track_lengths else 0,
            "max_track_length": max(track_lengths) if track_lengths else 0
        }
    
    def export_to_json(self, filepath: str):
        """Export reconstruction to JSON file.
        
        Args:
            filepath: Output JSON file path
        """
        data = {
            "cameras": {},
            "points_3d": {},
            "summary": self.get_structure_summary()
        }
        
        # Export cameras
        for img_id, cam in self.cameras.items():
            if cam.registered:
                data["cameras"][img_id] = {
                    "R": cam.R.tolist(),
                    "t": cam.t.ravel().tolist(),
                    "K": cam.K.tolist()
                }
        
        # Export 3D points
        for point_id, point in self.points_3d.items():
            data["points_3d"][str(point_id)] = {
                "xyz": point.xyz.tolist(),
                "observations": {
                    img_id: pt_2d.tolist()
                    for img_id, pt_2d in point.observations.items()
                },
                "error": float(point.error),
                "n_views": len(point.observations)
            }
        
        # Write to file
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    def load_from_json(self, filepath: str):
        """Load reconstruction from JSON file.
        
        Args:
            filepath: Input JSON file path
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        # Load cameras
        self.cameras = {}
        for img_id, cam_data in data["cameras"].items():
            self.cameras[img_id] = Camera(
                image_id=img_id,
                R=np.array(cam_data["R"]),
                t=np.array(cam_data["t"]).reshape(3, 1),
                K=np.array(cam_data["K"]),
                registered=True
            )
        
        # Load 3D points
        self.points_3d = {}
        self._next_point_id = 0
        
        for point_id_str, point_data in data["points_3d"].items():
            point_id = int(point_id_str)
            self._next_point_id = max(self._next_point_id, point_id + 1)
            
            observations = {
                img_id: np.array(pt_2d)
                for img_id, pt_2d in point_data["observations"].items()
            }
            
            self.points_3d[point_id] = Point3D(
                point_id=point_id,
                xyz=np.array(point_data["xyz"]),
                observations=observations,
                error=point_data.get("error", 0.0)
            )
