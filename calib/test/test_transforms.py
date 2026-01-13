"""
Unit tests for SE3Transform and Sim3Transform classes.

Tests cover:
- Identity transforms
- Inverse correctness
- Composition correctness
- Umeyama algorithm validation
- Save/load round-trip
- Rotation matrix validation
"""

import pytest
import numpy as np
from pathlib import Path
import tempfile
import json

from src.transforms import (
    SE3Transform,
    Sim3Transform,
    umeyama,
    compute_alignment
)


class TestSE3Transform:
    """Test SE3 rigid transforms."""
    
    def test_identity(self):
        """Test identity transform."""
        T = SE3Transform(R=np.eye(3), t=np.zeros(3))
        
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        result = T.apply(points)
        
        np.testing.assert_allclose(result, points, atol=1e-10)
    
    def test_translation_only(self):
        """Test pure translation."""
        t = np.array([10.0, 20.0, 30.0])
        T = SE3Transform(R=np.eye(3), t=t)
        
        points = np.array([[1.0, 2.0, 3.0]])
        result = T.apply(points)
        
        expected = points + t
        np.testing.assert_allclose(result, expected, atol=1e-10)
    
    def test_rotation_90deg_z(self):
        """Test 90-degree rotation around Z-axis."""
        # 90 deg rotation around Z: (x, y, z) -> (-y, x, z)
        R = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        T = SE3Transform(R=R, t=np.zeros(3))
        
        points = np.array([[1.0, 0.0, 0.0]])
        result = T.apply(points)
        
        expected = np.array([[0.0, 1.0, 0.0]])
        np.testing.assert_allclose(result, expected, atol=1e-10)
    
    def test_inverse(self):
        """Test inverse transform."""
        R = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        t = np.array([10.0, 20.0, 30.0])
        T = SE3Transform(R=R, t=t)
        T_inv = T.inverse()
        
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        
        # Apply forward then inverse should return original
        transformed = T.apply(points)
        recovered = T_inv.apply(transformed)
        
        np.testing.assert_allclose(recovered, points, atol=1e-10)
    
    def test_compose(self):
        """Test transform composition."""
        # T1: rotate 90 deg around Z
        R1 = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        t1 = np.array([1.0, 0.0, 0.0])
        T1 = SE3Transform(R=R1, t=t1, source_frame="A", target_frame="B")
        
        # T2: translate in Y
        R2 = np.eye(3)
        t2 = np.array([0.0, 5.0, 0.0])
        T2 = SE3Transform(R=R2, t=t2, source_frame="B", target_frame="C")
        
        # Compose: T_C_from_A = T2 @ T1
        T_composed = T2.compose(T1)
        
        assert T_composed.source_frame == "A"
        assert T_composed.target_frame == "C"
        
        # Test on points
        points = np.array([[1.0, 0.0, 0.0]])
        
        # Method 1: Compose then apply
        result1 = T_composed.apply(points)
        
        # Method 2: Apply sequentially
        temp = T1.apply(points)
        result2 = T2.apply(temp)
        
        np.testing.assert_allclose(result1, result2, atol=1e-10)
    
    def test_invalid_rotation_det(self):
        """Test that invalid rotation (det != 1) raises error."""
        R = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0]  # Reflection: det = -1
        ])
        
        with pytest.raises(ValueError, match="det\\(R\\)"):
            SE3Transform(R=R, t=np.zeros(3))
    
    def test_invalid_rotation_orthonormal(self):
        """Test that non-orthonormal matrix raises error."""
        R = np.array([
            [1.0, 0.1, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        with pytest.raises(ValueError, match="not orthonormal"):
            SE3Transform(R=R, t=np.zeros(3))
    
    def test_save_load(self):
        """Test save/load round-trip."""
        R = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        t = np.array([10.0, 20.0, 30.0])
        T = SE3Transform(R=R, t=t, source_frame="L", target_frame="U")
        
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = Path(tmpdir) / "transform.json"
            T.save(str(filepath), rmse_mm=0.123, n_points=16)
            
            # Check file exists
            assert filepath.exists()
            
            # Load and verify
            T_loaded = SE3Transform.load(str(filepath))
            
            np.testing.assert_allclose(T_loaded.R, T.R, atol=1e-10)
            np.testing.assert_allclose(T_loaded.t, T.t, atol=1e-10)
            assert T_loaded.source_frame == "L"
            assert T_loaded.target_frame == "U"


class TestSim3Transform:
    """Test Sim3 similarity transforms."""
    
    def test_identity(self):
        """Test identity Sim3 transform."""
        T = Sim3Transform(s=1.0, R=np.eye(3), t=np.zeros(3))
        
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        result = T.apply(points)
        
        np.testing.assert_allclose(result, points, atol=1e-10)
    
    def test_scale_only(self):
        """Test pure scaling."""
        T = Sim3Transform(s=2.0, R=np.eye(3), t=np.zeros(3))
        
        points = np.array([[1.0, 2.0, 3.0]])
        result = T.apply(points)
        
        expected = 2.0 * points
        np.testing.assert_allclose(result, expected, atol=1e-10)
    
    def test_inverse(self):
        """Test Sim3 inverse."""
        R = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        T = Sim3Transform(s=2.5, R=R, t=np.array([10.0, 20.0, 30.0]))
        T_inv = T.inverse()
        
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        
        transformed = T.apply(points)
        recovered = T_inv.apply(transformed)
        
        np.testing.assert_allclose(recovered, points, atol=1e-9)
    
    def test_negative_scale_raises(self):
        """Test that negative scale raises error."""
        with pytest.raises(ValueError, match="Scale must be positive"):
            Sim3Transform(s=-1.0, R=np.eye(3), t=np.zeros(3))
    
    def test_save_load(self):
        """Test Sim3 save/load round-trip."""
        R = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        T = Sim3Transform(s=1.23, R=R, t=np.array([10.0, 20.0, 30.0]))
        
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = Path(tmpdir) / "sim3.json"
            T.save(str(filepath))
            
            T_loaded = Sim3Transform.load(str(filepath))
            
            np.testing.assert_allclose(T_loaded.s, T.s, atol=1e-10)
            np.testing.assert_allclose(T_loaded.R, T.R, atol=1e-10)
            np.testing.assert_allclose(T_loaded.t, T.t, atol=1e-10)


class TestUmeyama:
    """Test Umeyama algorithm for optimal alignment."""
    
    def test_identity_alignment(self):
        """Test alignment of identical point sets."""
        src = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        dst = src.copy()
        
        scale, R, t, rmse = umeyama(src, dst, estimate_scale=False)
        
        assert abs(scale - 1.0) < 1e-10
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)
        np.testing.assert_allclose(t, np.zeros((3, 1)), atol=1e-10)
        assert rmse < 1e-10
    
    def test_translation_only(self):
        """Test pure translation alignment."""
        src = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        translation = np.array([10.0, 20.0, 30.0])
        dst = src + translation
        
        scale, R, t, rmse = umeyama(src, dst, estimate_scale=False)
        
        assert abs(scale - 1.0) < 1e-10
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)
        np.testing.assert_allclose(t.flatten(), translation, atol=1e-10)
        assert rmse < 1e-9
    
    def test_rotation_90deg(self):
        """Test 90-degree rotation alignment."""
        src = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0]
        ])
        
        # 90 deg rotation around Z
        R_true = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        dst = (R_true @ src.T).T
        
        scale, R, t, rmse = umeyama(src, dst, estimate_scale=False)
        
        np.testing.assert_allclose(R, R_true, atol=1e-10)
        assert rmse < 1e-9
    
    def test_scale_estimation(self):
        """Test scale estimation with Sim(3)."""
        src = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        scale_true = 2.5
        dst = scale_true * src + np.array([5.0, 10.0, 15.0])
        
        scale, R, t, rmse = umeyama(src, dst, estimate_scale=True)
        
        assert abs(scale - scale_true) < 1e-10
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)
        assert rmse < 1e-9
    
    def test_full_sim3(self):
        """Test full Sim(3) transform (scale + rotation + translation)."""
        src = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0]
        ])
        
        scale_true = 1.5
        R_true = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        t_true = np.array([[10.0], [20.0], [30.0]])
        
        dst = (scale_true * R_true @ src.T).T + t_true.T
        
        scale, R, t, rmse = umeyama(src, dst, estimate_scale=True)
        
        assert abs(scale - scale_true) < 1e-10
        np.testing.assert_allclose(R, R_true, atol=1e-10)
        np.testing.assert_allclose(t, t_true, atol=1e-9)
        assert rmse < 1e-9


class TestComputeAlignment:
    """Test high-level compute_alignment function."""
    
    def test_basic_alignment(self):
        """Test basic point set alignment."""
        src_points = {
            "p1": np.array([0.0, 0.0, 0.0]),
            "p2": np.array([1.0, 0.0, 0.0]),
            "p3": np.array([0.0, 1.0, 0.0]),
            "p4": np.array([0.0, 0.0, 1.0])
        }
        
        # Apply known transform
        R_true = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        t_true = np.array([5.0, 10.0, 15.0])
        
        dst_points = {
            pid: (R_true @ pos + t_true)
            for pid, pos in src_points.items()
        }
        
        # Compute alignment
        transform, rmse, residuals = compute_alignment(
            src_points, dst_points, estimate_scale=False
        )
        
        assert rmse < 1e-9
        np.testing.assert_allclose(transform.R, R_true, atol=1e-10)
        np.testing.assert_allclose(transform.t.flatten(), t_true, atol=1e-9)
        
        # Check all residuals are small
        for residual in residuals.values():
            assert residual < 1e-9
    
    def test_no_common_points_raises(self):
        """Test that no common points raises error."""
        src = {"p1": np.array([0.0, 0.0, 0.0])}
        dst = {"p2": np.array([1.0, 1.0, 1.0])}
        
        with pytest.raises(ValueError, match="No common points"):
            compute_alignment(src, dst)
    
    def test_rmse_fail_threshold(self):
        """Test RMSE failure threshold."""
        src_points = {
            "p1": np.array([0.0, 0.0, 0.0]),
            "p2": np.array([1.0, 0.0, 0.0]),
            "p3": np.array([0.0, 1.0, 0.0])
        }
        
        # Add large noise to destination
        dst_points = {
            pid: pos + np.random.randn(3) * 10.0
            for pid, pos in src_points.items()
        }
        
        with pytest.raises(ValueError, match="RMSE.*exceeds failure threshold"):
            compute_alignment(src_points, dst_points, rmse_fail_threshold=0.1)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
