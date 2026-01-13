"""
Integration tests for Phase 0: Triangulation + U-frame definition.

Tests the complete workflow:
1. Synthetic 3D points (reference plate corners)
2. Project to multiple camera views
3. Triangulate back to 3D (L-frame)
4. Align L-frame to U-frame via reference plate
5. Validate round-trip accuracy

This validates the core Phase 0 pipeline without real images.
"""

import pytest
import numpy as np
from pathlib import Path
import tempfile
import json

from tools.triangulation import (
    triangulate_points,
    build_projection_matrix,
    cameras_to_projection_matrices
)
from tools.bundle_adjustment import export_refpoints_L, load_refpoints_L
from tools.define_user_frame import define_user_frame
from src.transforms import SE3Transform


def generate_camera_poses(n_cameras: int = 5, radius: float = 200.0) -> dict:
    """Generate synthetic camera poses in a circular pattern.
    
    Args:
        n_cameras: Number of cameras
        radius: Distance from origin in mm
        
    Returns:
        Dict mapping camera_id to {"R": 3x3, "t": 3x1}
    """
    cameras = {}
    
    for i in range(n_cameras):
        angle = 2 * np.pi * i / n_cameras
        
        # Camera position
        cam_pos = np.array([
            radius * np.cos(angle),
            radius * np.sin(angle),
            100.0  # Above XY plane
        ])
        
        # Look at origin
        z_axis = -cam_pos / np.linalg.norm(cam_pos)
        
        # Up vector (world Z)
        up = np.array([0.0, 0.0, 1.0])
        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        
        y_axis = np.cross(z_axis, x_axis)
        
        # Rotation matrix (world to camera)
        R = np.vstack([x_axis, y_axis, z_axis])
        
        # Translation (camera position in world frame)
        t = -R @ cam_pos.reshape(3, 1)
        
        cameras[i] = {"R": R, "t": t}
    
    return cameras


def generate_intrinsics(
    fx: float = 3000.0,
    fy: float = 3000.0,
    cx: float = 3000.0,
    cy: float = 2000.0
) -> np.ndarray:
    """Generate camera intrinsic matrix.
    
    Args:
        fx, fy: Focal lengths in pixels
        cx, cy: Principal point in pixels
        
    Returns:
        3x3 intrinsic matrix K
    """
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0]
    ])


def project_points(
    points_3d: dict,
    cameras: dict,
    K: np.ndarray,
    noise_std: float = 0.0
) -> dict:
    """Project 3D points to all camera views.
    
    Args:
        points_3d: Dict mapping point IDs to 3D positions (mm)
        cameras: Dict mapping camera_id to {"R", "t"}
        K: 3x3 intrinsic matrix
        noise_std: Gaussian noise std dev in pixels
        
    Returns:
        Dict mapping point IDs to list of (camera_id, pixel_coords)
    """
    observations = {pid: [] for pid in points_3d.keys()}
    
    for cam_id, camera in cameras.items():
        R = np.array(camera["R"])
        t = np.array(camera["t"]).reshape(3, 1)
        
        for point_id, point_3d in points_3d.items():
            # Transform to camera frame
            p_cam = R @ point_3d.reshape(3, 1) + t
            
            # Skip if behind camera
            if p_cam[2, 0] <= 0:
                continue
            
            # Project to image
            p_img = K @ p_cam
            pixel = (p_img[:2] / p_img[2]).flatten()
            
            # Add noise
            if noise_std > 0:
                pixel += np.random.randn(2) * noise_std
            
            observations[point_id].append((cam_id, pixel))
    
    return observations


class TestTriangulationAccuracy:
    """Test triangulation with synthetic data."""
    
    def test_perfect_triangulation(self):
        """Test triangulation with noise-free observations."""
        # Generate reference plate points (U-frame)
        points_U_true = {
            "1_TL": np.array([0.0, 0.0, 0.0]),
            "1_TR": np.array([8.8, 0.0, 0.0]),
            "1_BR": np.array([8.8, -8.8, 0.0]),
            "1_BL": np.array([0.0, -8.8, 0.0])
        }
        
        # Generate camera poses
        cameras = generate_camera_poses(n_cameras=6)
        K = generate_intrinsics()
        
        # Project to images (noise-free)
        observations = project_points(points_U_true, cameras, K, noise_std=0.0)
        
        # Build projection matrices
        P_matrices = cameras_to_projection_matrices(cameras, K)
        
        # Triangulate
        triangulated = triangulate_points(
            observations, P_matrices, min_views=2, refine=True
        )
        
        # Validate: should recover original points exactly
        for point_id in points_U_true.keys():
            assert point_id in triangulated
            
            pos_recovered = np.array(triangulated[point_id]["position_mm"])
            pos_true = points_U_true[point_id]
            
            error = np.linalg.norm(pos_recovered - pos_true)
            assert error < 1e-6, f"{point_id}: error {error:.2e} mm"
            
            # Check reprojection error
            reproj_error = triangulated[point_id]["mean_reprojection_error_px"]
            assert reproj_error < 1e-6, f"{point_id}: reproj error {reproj_error:.2e} px"
    
    def test_noisy_triangulation(self):
        """Test triangulation with realistic pixel noise."""
        points_U_true = {
            "1_TL": np.array([0.0, 0.0, 0.0]),
            "1_TR": np.array([8.8, 0.0, 0.0]),
            "1_BR": np.array([8.8, -8.8, 0.0]),
            "1_BL": np.array([0.0, -8.8, 0.0]),
            "2_TL": np.array([15.0, 0.0, 0.0]),
            "2_TR": np.array([23.8, 0.0, 0.0])
        }
        
        cameras = generate_camera_poses(n_cameras=8)
        K = generate_intrinsics()
        
        # Add 0.3 px Gaussian noise
        observations = project_points(points_U_true, cameras, K, noise_std=0.3)
        P_matrices = cameras_to_projection_matrices(cameras, K)
        
        triangulated = triangulate_points(
            observations, P_matrices, min_views=3, refine=True
        )
        
        # With 0.3 px noise, expect ~0.01-0.1 mm 3D error
        for point_id in points_U_true.keys():
            pos_recovered = np.array(triangulated[point_id]["position_mm"])
            pos_true = points_U_true[point_id]
            
            error = np.linalg.norm(pos_recovered - pos_true)
            assert error < 0.1, f"{point_id}: error {error:.3f} mm (expected < 0.1 mm)"
            
            reproj_error = triangulated[point_id]["mean_reprojection_error_px"]
            assert reproj_error < 1.0, f"{point_id}: reproj {reproj_error:.2f} px"


class TestPhase0Pipeline:
    """Integration test for complete Phase 0 workflow."""
    
    def test_full_pipeline_noise_free(self):
        """Test complete pipeline: triangulate L-frame → align to U-frame."""
        
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)
            
            # Step 1: Define true U-frame points (reference plate)
            points_U_true = {
                "1_TL": np.array([0.0, 0.0, 0.0]),
                "1_TR": np.array([8.8, 0.0, 0.0]),
                "1_BR": np.array([8.8, -8.8, 0.0]),
                "1_BL": np.array([0.0, -8.8, 0.0]),
                "2_TL": np.array([15.0, 0.0, 0.0]),
                "2_TR": np.array([23.8, 0.0, 0.0]),
                "2_BR": np.array([23.8, -8.8, 0.0]),
                "2_BL": np.array([15.0, -8.8, 0.0])
            }
            
            # Step 2: Generate cameras and project
            cameras = generate_camera_poses(n_cameras=10)
            K = generate_intrinsics()
            observations = project_points(points_U_true, cameras, K, noise_std=0.0)
            P_matrices = cameras_to_projection_matrices(cameras, K)
            
            # Step 3: Triangulate (produces L-frame, which should equal U-frame here)
            triangulated = triangulate_points(
                observations, P_matrices, min_views=4, refine=True
            )
            
            # Step 4: Export refpoints_L.json
            refpoints_L_path = tmpdir / "refpoints_L.json"
            export_refpoints_L(triangulated, str(refpoints_L_path))
            
            # Verify file was created
            assert refpoints_L_path.exists()
            
            # Step 5: Create reference plate fixture
            reference_plate_path = tmpdir / "reference_plate.json"
            reference_data = {
                "frame": "U",
                "units": "mm",
                "points": {pid: pos.tolist() for pid, pos in points_U_true.items()}
            }
            with open(reference_plate_path, 'w') as f:
                json.dump(reference_data, f)
            
            # Step 6: Define U-frame (compute T_U_from_L)
            T_U_from_L_path = tmpdir / "T_U_from_L.json"
            T_U_from_L, rmse = define_user_frame(
                str(refpoints_L_path),
                str(reference_plate_path),
                estimate_scale=False,
                output_path=str(T_U_from_L_path)
            )
            
            # Step 7: Validate alignment
            # Since L-frame == U-frame in this synthetic case, expect identity
            assert rmse < 1e-6, f"RMSE {rmse:.2e} mm (expected ~0)"
            
            # Check transform is near identity
            R_diff = np.max(np.abs(T_U_from_L.R - np.eye(3)))
            t_norm = np.linalg.norm(T_U_from_L.t)
            
            assert R_diff < 1e-6, f"R differs from identity by {R_diff:.2e}"
            assert t_norm < 1e-6, f"t = {t_norm:.2e} mm (expected ~0)"
            
            print(f"✓ Phase 0 pipeline test passed (RMSE: {rmse:.2e} mm)")
    
    def test_pipeline_with_arbitrary_L_frame(self):
        """Test pipeline when L-frame differs from U-frame."""
        
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)
            
            # Define U-frame (reference)
            points_U_true = {
                "1_TL": np.array([0.0, 0.0, 0.0]),
                "1_TR": np.array([8.8, 0.0, 0.0]),
                "1_BR": np.array([8.8, -8.8, 0.0]),
                "1_BL": np.array([0.0, -8.8, 0.0])
            }
            
            # Apply arbitrary transform to create L-frame
            R_L_from_U = np.array([
                [0.0, -1.0, 0.0],
                [1.0,  0.0, 0.0],
                [0.0,  0.0, 1.0]
            ])
            t_L_from_U = np.array([50.0, 100.0, 150.0])
            
            points_L_true = {
                pid: R_L_from_U @ pos + t_L_from_U
                for pid, pos in points_U_true.items()
            }
            
            # Generate observations and triangulate
            cameras = generate_camera_poses(n_cameras=8, radius=300.0)
            K = generate_intrinsics()
            observations = project_points(points_L_true, cameras, K, noise_std=0.2)
            P_matrices = cameras_to_projection_matrices(cameras, K)
            
            triangulated = triangulate_points(
                observations, P_matrices, min_views=3, refine=True
            )
            
            # Export L-frame
            refpoints_L_path = tmpdir / "refpoints_L.json"
            export_refpoints_L(triangulated, str(refpoints_L_path))
            
            # Create reference plate
            reference_plate_path = tmpdir / "reference_plate.json"
            reference_data = {
                "frame": "U",
                "units": "mm",
                "points": {pid: pos.tolist() for pid, pos in points_U_true.items()}
            }
            with open(reference_plate_path, 'w') as f:
                json.dump(reference_data, f)
            
            # Compute T_U_from_L
            T_U_from_L, rmse = define_user_frame(
                str(refpoints_L_path),
                str(reference_plate_path),
                estimate_scale=False
            )
            
            # Validate: RMSE should be very small
            assert rmse < 0.05, f"RMSE {rmse:.3f} mm (expected < 0.05 mm)"
            
            # Validate: recovered transform should match ground truth
            T_U_from_L_true = SE3Transform(
                R=R_L_from_U.T,
                t=-R_L_from_U.T @ t_L_from_U
            )
            
            R_error = np.max(np.abs(T_U_from_L.R - T_U_from_L_true.R))
            t_error = np.linalg.norm(T_U_from_L.t - T_U_from_L_true.t)
            
            assert R_error < 0.01, f"R error: {R_error:.2e}"
            assert t_error < 0.2, f"t error: {t_error:.3f} mm (with 0.2px noise)"
            
            print(f"✓ Arbitrary L-frame test passed (RMSE: {rmse:.3f} mm)")


class TestMinimumViews:
    """Test minimum view requirements for triangulation."""
    
    def test_two_views_minimum(self):
        """Test that 2 views is sufficient for triangulation."""
        points_3d = {"p1": np.array([0.0, 0.0, 0.0])}
        
        cameras = generate_camera_poses(n_cameras=2)
        K = generate_intrinsics()
        observations = project_points(points_3d, cameras, K)
        P_matrices = cameras_to_projection_matrices(cameras, K)
        
        triangulated = triangulate_points(
            observations, P_matrices, min_views=2
        )
        
        assert "p1" in triangulated
        assert triangulated["p1"]["n_views"] == 2
    
    def test_insufficient_views_skipped(self):
        """Test that points with too few views are skipped."""
        # Create point visible in only 1 camera
        points_3d = {"p1": np.array([0.0, 0.0, 0.0])}
        
        # Use only 1 camera
        cameras = {0: generate_camera_poses(n_cameras=1)[0]}
        K = generate_intrinsics()
        observations = project_points(points_3d, cameras, K)
        P_matrices = cameras_to_projection_matrices(cameras, K)
        
        triangulated = triangulate_points(
            observations, P_matrices, min_views=2
        )
        
        # Should be skipped due to insufficient views
        assert "p1" not in triangulated


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
