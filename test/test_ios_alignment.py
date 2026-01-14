"""
Unit tests for IOS frame alignment (T_I_from_U).

Tests:
    - Implant loading and validation
    - Implant matching by marker_id
    - Alignment computation with known transformation
    - RMSE validation
    - Transform application to implants

Run:
    pytest test/test_ios_alignment.py -v
"""

import pytest
import numpy as np
import json
import tempfile
from pathlib import Path
import sys

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
from transforms import SE3Transform, compute_alignment

# Import functions from solve_T_I_from_U
sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))
from solve_T_I_from_U import (
    load_implants,
    match_implants,
    compute_alignment_from_implants,
    transform_implants_to_i_frame,
    compute_rmse
)


class TestImplantLoading:
    """Test loading implants from JSON files."""
    
    @pytest.fixture
    def implants_json(self, tmp_path):
        """Create mock implants JSON."""
        data = {
            "frame": "U",
            "units": "mm",
            "implants": {
                "1": {
                    "centroid_mm": [10.0, 20.0, 30.0],
                    "axis_vector": [0.0, 0.0, 1.0],
                    "marker_id": 100
                },
                "2": {
                    "centroid_mm": [15.0, 25.0, 35.0],
                    "axis_vector": [0.0, 0.1, 0.995],
                    "marker_id": 101
                }
            },
            "metadata": {
                "timestamp": "2026-01-14T10:00:00Z",
                "source": "test"
            }
        }
        
        path = tmp_path / "implants.json"
        with open(path, 'w') as f:
            json.dump(data, f)
        
        return str(path)
    
    def test_load_valid_implants(self, implants_json):
        """Test loading valid implants file."""
        data, centroids, axes, ids = load_implants(implants_json)
        
        assert len(ids) == 2
        assert ids == ["1", "2"]
        assert centroids.shape == (2, 3)
        assert axes.shape == (2, 3)
        assert np.allclose(centroids[0], [10.0, 20.0, 30.0])
    
    def test_invalid_units(self, tmp_path):
        """Test error on non-mm units."""
        data = {
            "frame": "U",
            "units": "inches",
            "implants": {}
        }
        
        path = tmp_path / "bad_units.json"
        with open(path, 'w') as f:
            json.dump(data, f)
        
        with pytest.raises(ValueError, match="Expected mm units"):
            load_implants(str(path))
    
    def test_missing_centroid(self, tmp_path):
        """Test error on missing centroid_mm field."""
        data = {
            "frame": "U",
            "units": "mm",
            "implants": {
                "1": {
                    "axis_vector": [0.0, 0.0, 1.0],
                    "marker_id": 100
                }
            }
        }
        
        path = tmp_path / "no_centroid.json"
        with open(path, 'w') as f:
            json.dump(data, f)
        
        with pytest.raises(ValueError, match="missing 'centroid_mm'"):
            load_implants(str(path))


class TestImplantMatching:
    """Test matching implants between U and I frames."""
    
    def test_match_all_implants(self):
        """Test matching when all implants have correspondences."""
        u_data = {
            "implants": {
                "1": {"marker_id": 100},
                "2": {"marker_id": 101},
                "3": {"marker_id": 102}
            }
        }
        
        i_data = {
            "implants": {
                "10": {"marker_id": 100},
                "20": {"marker_id": 101},
                "30": {"marker_id": 102}
            }
        }
        
        u_ids, i_ids, markers = match_implants(u_data, i_data)
        
        assert len(u_ids) == 3
        assert len(i_ids) == 3
        assert markers == [100, 101, 102]
        assert u_ids == ["1", "2", "3"]
        assert i_ids == ["10", "20", "30"]
    
    def test_partial_matches(self):
        """Test matching when only subset of implants match."""
        u_data = {
            "implants": {
                "1": {"marker_id": 100},
                "2": {"marker_id": 101},
                "3": {"marker_id": 102},
                "4": {"marker_id": 103}
            }
        }
        
        i_data = {
            "implants": {
                "10": {"marker_id": 100},
                "20": {"marker_id": 102}
                # Missing 101 and 103
            }
        }
        
        u_ids, i_ids, markers = match_implants(u_data, i_data)
        
        assert len(markers) == 2
        assert markers == [100, 102]
        assert u_ids == ["1", "3"]
    
    def test_no_matches(self):
        """Test error when no implants match."""
        u_data = {
            "implants": {
                "1": {"marker_id": 100}
            }
        }
        
        i_data = {
            "implants": {
                "10": {"marker_id": 200}
            }
        }
        
        with pytest.raises(ValueError, match="No matching implants"):
            match_implants(u_data, i_data)


class TestAlignmentComputation:
    """Test alignment computation with known transformations."""
    
    def test_identity_alignment(self):
        """Test alignment when U and I frames are identical."""
        # 4 points in a square
        points = np.array([
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [10.0, 10.0, 0.0],
            [0.0, 10.0, 0.0]
        ])
        
        T = compute_alignment_from_implants(points, points)
        
        assert np.allclose(T.R, np.eye(3), atol=1e-6)
        assert np.allclose(T.t, [0.0, 0.0, 0.0], atol=1e-6)
    
    def test_translation_only(self):
        """Test alignment with pure translation."""
        src = np.array([
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0]
        ])
        
        # Translate by [5, -3, 2]
        translation = np.array([5.0, -3.0, 2.0])
        dst = src + translation
        
        T = compute_alignment_from_implants(src, dst)
        
        assert np.allclose(T.R, np.eye(3), atol=1e-6)
        assert np.allclose(T.t.flatten(), translation, atol=1e-6)
    
    def test_rotation_90deg_z(self):
        """Test alignment with 90° rotation around Z axis."""
        src = np.array([
            [10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 10.0]
        ])
        
        # 90° rotation around Z
        R_90 = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        dst = (R_90 @ src.T).T
        
        T = compute_alignment_from_implants(src, dst)
        
        assert np.allclose(T.R, R_90, atol=1e-6)
        assert np.allclose(T.t, [0.0, 0.0, 0.0], atol=1e-6)
    
    def test_rigid_transform_recovery(self):
        """Test recovery of known rigid transformation."""
        # Generate random points
        np.random.seed(42)
        src = np.random.randn(5, 3) * 10.0
        
        # Apply known transformation
        angle = np.pi / 6  # 30°
        R_known = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0]
        ])
        t_known = np.array([5.0, -2.0, 3.0])
        
        dst = (R_known @ src.T).T + t_known
        
        # Recover transformation
        T = compute_alignment_from_implants(src, dst)
        
        assert np.allclose(T.R, R_known, atol=1e-6)
        assert np.allclose(T.t.flatten(), t_known, atol=1e-6)
    
    def test_insufficient_correspondences(self):
        """Test error with < 3 points."""
        src = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        dst = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        
        with pytest.raises(ValueError, match="at least 3 correspondences"):
            compute_alignment_from_implants(src, dst)


class TestRMSEComputation:
    """Test RMSE and residual computation."""
    
    def test_zero_rmse_identical_points(self):
        """Test RMSE=0 for identical point sets."""
        points = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0]
        ])
        
        rmse, residuals = compute_rmse(points, points)
        
        assert np.isclose(rmse, 0.0, atol=1e-10)
        assert np.allclose(residuals, 0.0, atol=1e-10)
    
    def test_known_rmse(self):
        """Test RMSE with known displacement."""
        src = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
        ])
        
        # Offset by 3mm in X (residuals = [3.0, 3.0])
        dst = np.array([
            [3.0, 0.0, 0.0],
            [4.0, 0.0, 0.0]
        ])
        
        rmse, residuals = compute_rmse(src, dst)
        
        assert np.isclose(rmse, 3.0, atol=1e-6)
        assert np.allclose(residuals, [3.0, 3.0], atol=1e-6)
    
    def test_rmse_mixed_residuals(self):
        """Test RMSE with varying residuals."""
        src = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ])
        
        # Residuals: 0mm, 3mm, 4mm → RMSE = sqrt((0 + 9 + 16)/3) = sqrt(25/3) ≈ 2.887
        dst = np.array([
            [0.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [4.0, 0.0, 0.0]
        ])
        
        rmse, residuals = compute_rmse(src, dst)
        
        expected_rmse = np.sqrt((0**2 + 3**2 + 4**2) / 3)
        assert np.isclose(rmse, expected_rmse, atol=1e-6)
        assert np.allclose(residuals, [0.0, 3.0, 4.0], atol=1e-6)


class TestTransformApplication:
    """Test applying transformation to implants."""
    
    def test_transform_implants(self):
        """Test transforming implants from U to I frame."""
        u_data = {
            "implants": {
                "1": {
                    "centroid_mm": [10.0, 0.0, 0.0],
                    "axis_vector": [1.0, 0.0, 0.0],
                    "marker_id": 100,
                    "reprojection_error_px": 0.5
                },
                "2": {
                    "centroid_mm": [0.0, 10.0, 0.0],
                    "axis_vector": [0.0, 1.0, 0.0],
                    "marker_id": 101,
                    "reprojection_error_px": 0.3
                }
            }
        }
        
        # 90° rotation around Z + translation [5, 5, 0]
        R = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        t = np.array([5.0, 5.0, 0.0])
        T = SE3Transform(R=R, t=t)
        
        transformed = transform_implants_to_i_frame(u_data, T, ["1", "2"])
        
        # Check transformed centroids
        # [10, 0, 0] → [0, 10, 0] + [5, 5, 0] = [5, 15, 0]
        assert np.allclose(transformed["1"]["centroid_mm"], [5.0, 15.0, 0.0], atol=1e-6)
        
        # [0, 10, 0] → [-10, 0, 0] + [5, 5, 0] = [-5, 5, 0]
        assert np.allclose(transformed["2"]["centroid_mm"], [-5.0, 5.0, 0.0], atol=1e-6)
        
        # Check transformed axes
        # [1, 0, 0] → [0, 1, 0]
        assert np.allclose(transformed["1"]["axis_vector"], [0.0, 1.0, 0.0], atol=1e-6)
        
        # [0, 1, 0] → [-1, 0, 0]
        assert np.allclose(transformed["2"]["axis_vector"], [-1.0, 0.0, 0.0], atol=1e-6)
        
        # Check metadata preserved
        assert transformed["1"]["marker_id"] == 100
        assert transformed["1"]["reprojection_error_px"] == 0.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
