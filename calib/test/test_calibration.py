"""
Unit tests for camera calibration tools.

Tests cover:
- Calibration file loading and parsing
- Undistort/redistort round-trip accuracy
- CalibrationQA validation checks
- JSON schema compliance
"""

import pytest
import numpy as np
import json
from pathlib import Path
import tempfile

from src.calibration_loader import (
    load_calibration,
    undistort_points,
    redistort_points,
    get_magnification,
    CalibrationQA
)


@pytest.fixture
def sample_calibration():
    """Create a sample calibration dict for testing."""
    return {
        "calibration_type": "charuco",
        "camera_model": "Nikon D5600",
        "lens": "50mm",
        "aperture": "f/16",
        "timestamp": "2026-01-13T10:00:00Z",
        "image_size": {
            "width": 6000,
            "height": 4000
        },
        "intrinsics": {
            "fx": 5234.56,
            "fy": 5238.91,
            "cx": 3012.34,
            "cy": 2005.67,
            "K": [[5234.56, 0, 3012.34], [0, 5238.91, 2005.67], [0, 0, 1]]
        },
        "distortion": {
            "k1": -0.123,
            "k2": 0.045,
            "p1": 0.001,
            "p2": -0.002,
            "k3": -0.012,
            "coefficients": [-0.123, 0.045, 0.001, -0.002, -0.012]
        },
        "magnification": {
            "median_px_per_mm": 42.3,
            "min_px_per_mm": 38.1,
            "max_px_per_mm": 46.7
        },
        "quality": {
            "reprojection_error_px": 0.28,
            "n_images_used": 35,
            "n_unique_roll_angles": 8
        },
        "units": "mm"
    }


@pytest.fixture
def calibration_file(sample_calibration):
    """Create a temporary calibration JSON file."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(sample_calibration, f)
        filepath = f.name
    
    yield filepath
    
    # Cleanup
    Path(filepath).unlink()


class TestCalibrationLoader:
    """Test calibration file loading."""
    
    def test_load_calibration_success(self, calibration_file):
        """Test successful calibration loading."""
        K, D, image_size, metadata = load_calibration(calibration_file)
        
        # Check intrinsics
        assert K.shape == (3, 3)
        assert K[0, 0] == pytest.approx(5234.56)
        assert K[1, 1] == pytest.approx(5238.91)
        assert K[0, 2] == pytest.approx(3012.34)
        assert K[1, 2] == pytest.approx(2005.67)
        
        # Check distortion
        assert len(D) == 5
        assert D[0] == pytest.approx(-0.123)
        
        # Check image size
        assert image_size == (6000, 4000)
        
        # Check metadata
        assert metadata["camera_model"] == "Nikon D5600"
        assert metadata["reprojection_error_px"] == pytest.approx(0.28)
    
    def test_load_nonexistent_file(self):
        """Test loading nonexistent file raises error."""
        with pytest.raises(FileNotFoundError):
            load_calibration("nonexistent_file.json")
    
    def test_get_magnification(self, calibration_file):
        """Test magnification extraction."""
        mag = get_magnification(calibration_file)
        assert mag == pytest.approx(42.3)


class TestUndistortPoints:
    """Test point undistortion and round-trip accuracy."""
    
    def test_undistort_identity(self):
        """Test that undistortion with no distortion is identity-like."""
        K = np.array([
            [1000.0, 0.0, 500.0],
            [0.0, 1000.0, 400.0],
            [0.0, 0.0, 1.0]
        ])
        D = np.zeros(5)  # No distortion
        
        # Test points
        points = np.array([[500.0, 400.0], [600.0, 500.0]])
        
        # Undistort
        points_normalized = undistort_points(points, K, D, output="normalized")
        
        # With no distortion and points at principal point, should be near origin
        assert points_normalized[0, 0] == pytest.approx(0.0, abs=1e-6)
        assert points_normalized[0, 1] == pytest.approx(0.0, abs=1e-6)
    
    def test_round_trip_accuracy(self, calibration_file):
        """Test undistort→redistort round-trip accuracy."""
        K, D, image_size, _ = load_calibration(calibration_file)
        
        # Generate test points across image
        rng = np.random.RandomState(42)
        test_points = rng.uniform(
            low=[1000, 1000],
            high=[5000, 3000],
            size=(100, 2)
        )
        
        # Round-trip: distorted → normalized → distorted
        points_normalized = undistort_points(test_points, K, D, output="normalized")
        points_recovered = redistort_points(points_normalized, K, D)
        
        # Check accuracy
        errors = np.linalg.norm(test_points - points_recovered, axis=1)
        max_error = np.max(errors)
        mean_error = np.mean(errors)
        
        assert max_error < 1e-3, f"Max round-trip error {max_error:.2e} px"
        assert mean_error < 1e-6, f"Mean round-trip error {mean_error:.2e} px"
    
    def test_undistort_single_point(self):
        """Test undistortion of single point (1D input)."""
        K = np.array([
            [1000.0, 0.0, 500.0],
            [0.0, 1000.0, 400.0],
            [0.0, 0.0, 1.0]
        ])
        D = np.zeros(5)
        
        # Single point as 1D array
        point = np.array([550.0, 450.0])
        
        result = undistort_points(point, K, D, output="normalized")
        
        assert result.shape == (1, 2)


class TestCalibrationQA:
    """Test calibration quality assurance checks."""
    
    def test_qa_all_checks_pass(self, calibration_file):
        """Test that good calibration passes all QA checks."""
        K, D, image_size, metadata = load_calibration(calibration_file)
        
        qa = CalibrationQA(K, D, image_size, metadata)
        result = qa.run_all_checks()
        
        assert result is True
        assert len(qa.errors) == 0
    
    def test_qa_high_reprojection_error(self):
        """Test that high reprojection error is caught."""
        K = np.eye(3)
        K[0, 0] = K[1, 1] = 1000.0
        K[0, 2] = 500.0
        K[1, 2] = 400.0
        
        D = np.zeros(5)
        image_size = (1000, 800)
        
        metadata = {
            "reprojection_error_px": 1.5,  # High error
            "magnification_px_per_mm": 40.0,
            "n_images_used": 30
        }
        
        qa = CalibrationQA(K, D, image_size, metadata)
        qa.check_reprojection_error(max_error_px=0.5)
        
        assert len(qa.errors) > 0
        assert "Reprojection error" in qa.errors[0]
    
    def test_qa_principal_point_offset(self):
        """Test principal point offset warning."""
        K = np.eye(3)
        K[0, 0] = K[1, 1] = 1000.0
        K[0, 2] = 200.0  # Far from center (500, 400)
        K[1, 2] = 100.0
        
        D = np.zeros(5)
        image_size = (1000, 800)
        metadata = {"reprojection_error_px": 0.3}
        
        qa = CalibrationQA(K, D, image_size, metadata)
        qa.check_principal_point(max_offset_px=100.0)
        
        assert len(qa.warnings) > 0
        assert "Principal point offset" in qa.warnings[0]
    
    def test_qa_aspect_ratio(self):
        """Test aspect ratio check."""
        K = np.eye(3)
        K[0, 0] = 1000.0
        K[1, 1] = 1050.0  # Different from fx
        K[0, 2] = 500.0
        K[1, 2] = 400.0
        
        D = np.zeros(5)
        image_size = (1000, 800)
        metadata = {"reprojection_error_px": 0.3}
        
        qa = CalibrationQA(K, D, image_size, metadata)
        qa.check_aspect_ratio(max_deviation=0.01)
        
        # Aspect ratio = 1000/1050 ≈ 0.952, deviation > 0.01
        assert len(qa.warnings) > 0


class TestCalibrationFileSchema:
    """Test calibration file schema compliance."""
    
    def test_required_fields_present(self, sample_calibration):
        """Test that all required fields are present."""
        required_top_level = [
            "calibration_type", "image_size", "intrinsics", "distortion"
        ]
        
        for field in required_top_level:
            assert field in sample_calibration
        
        # Check intrinsics fields
        assert "fx" in sample_calibration["intrinsics"]
        assert "fy" in sample_calibration["intrinsics"]
        assert "cx" in sample_calibration["intrinsics"]
        assert "cy" in sample_calibration["intrinsics"]
        assert "K" in sample_calibration["intrinsics"]
        
        # Check distortion fields
        assert "coefficients" in sample_calibration["distortion"]
    
    def test_calibration_type_valid(self, sample_calibration):
        """Test calibration type is valid."""
        assert sample_calibration["calibration_type"] in ["charuco", "checkerboard", "apriltag"]
    
    def test_units_field(self, sample_calibration):
        """Test units field is present and correct."""
        assert sample_calibration["units"] == "mm"


class TestCharucoBoard:
    """Test ChArUco board configuration (basic validation)."""
    
    def test_charuco_board_spec_exists(self):
        """Test that ChArUco board specification file exists."""
        board_file = Path("calib/fixtures/charuco_board_9x6.json")
        assert board_file.exists(), "ChArUco board specification not found"
        
        with open(board_file, 'r') as f:
            data = json.load(f)
        
        # Validate structure
        assert data["board_type"] == "charuco"
        assert data["marker_family"] == "tag36h11"
        assert data["grid"]["squares_x"] == 9
        assert data["grid"]["squares_y"] == 6
        assert data["grid"]["square_size_mm"] == 10.0
        assert data["grid"]["marker_size_mm"] == 7.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
