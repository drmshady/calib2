"""
Unit tests for implant extraction from reference points.

Tests:
    - Centroid computation from tag corners
    - Axis vector (normal) computation
    - Tag geometry validation
    - End-to-end extraction pipeline

Run:
    pytest test/test_implant_extraction.py -v
"""

import pytest
import numpy as np
import json
import tempfile
from pathlib import Path
import sys

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
from implant_extractor import (
    compute_tag_plane_normal,
    compute_implant_centroid,
    compute_implant_axis,
    validate_tag_geometry,
    extract_implants_from_refpoints
)


class TestTagGeometry:
    """Test tag corner geometry computations."""
    
    def test_centroid_square_tag(self):
        """Test centroid of axis-aligned square tag."""
        # 8.8mm square centered at (10, 20, 30)
        corners = np.array([
            [5.6, 15.6, 30.0],  # TL
            [14.4, 15.6, 30.0],  # TR
            [14.4, 24.4, 30.0],  # BR
            [5.6, 24.4, 30.0]   # BL
        ])
        
        centroid = compute_implant_centroid(corners, tag_size_mm=8.8)
        
        assert np.allclose(centroid, [10.0, 20.0, 30.0], atol=1e-6)
    
    def test_axis_parallel_to_xy_plane(self):
        """Test axis normal for tag parallel to XY plane."""
        # Tag in XY plane at Z=10
        corners = np.array([
            [0.0, 0.0, 10.0],
            [8.8, 0.0, 10.0],
            [8.8, 8.8, 10.0],
            [0.0, 8.8, 10.0]
        ])
        
        axis = compute_implant_axis(corners)
        
        # Should point along Z axis (or -Z depending on winding)
        assert np.allclose(np.abs(axis[2]), 1.0, atol=1e-6)
        assert np.allclose(axis[0], 0.0, atol=1e-6)
        assert np.allclose(axis[1], 0.0, atol=1e-6)
    
    def test_axis_tilted_tag(self):
        """Test axis normal for tilted tag."""
        # Tag tilted 45° around X axis
        angle = np.pi / 4
        corners = np.array([
            [0.0, 0.0, 0.0],
            [8.8, 0.0, 0.0],
            [8.8, 8.8 * np.cos(angle), 8.8 * np.sin(angle)],
            [0.0, 8.8 * np.cos(angle), 8.8 * np.sin(angle)]
        ])
        
        axis = compute_implant_axis(corners)
        
        # Should point perpendicular to tilted plane
        expected = np.array([0.0, -np.sin(angle), np.cos(angle)])
        
        # Allow for sign flip
        assert np.allclose(np.abs(np.dot(axis, expected)), 1.0, atol=1e-3)
    
    def test_normal_degenerate_corners(self):
        """Test that collinear corners raise ValueError."""
        # All corners on a line
        corners = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0]
        ])
        
        with pytest.raises(ValueError, match="collinear"):
            compute_tag_plane_normal(corners)


class TestGeometryValidation:
    """Test tag geometry quality validation."""
    
    def test_valid_square_tag(self):
        """Test validation passes for perfect square."""
        corners = np.array([
            [0.0, 0.0, 0.0],
            [8.8, 0.0, 0.0],
            [8.8, 8.8, 0.0],
            [0.0, 8.8, 0.0]
        ])
        
        valid, msg = validate_tag_geometry(corners, tag_size_mm=8.8, tolerance_mm=0.5)
        
        assert valid
        assert "Valid" in msg
    
    def test_invalid_edge_length(self):
        """Test validation fails for incorrect edge length."""
        # 10mm edges instead of 8.8mm
        corners = np.array([
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [10.0, 10.0, 0.0],
            [0.0, 10.0, 0.0]
        ])
        
        valid, msg = validate_tag_geometry(corners, tag_size_mm=8.8, tolerance_mm=0.5)
        
        assert not valid
        assert "Edge length error" in msg
    
    def test_invalid_planarity(self):
        """Test validation fails for non-planar corners."""
        # One corner out of plane by 1mm (within edge tolerance but exceeds planarity)
        # Use tight tolerance to isolate planarity check
        corners = np.array([
            [0.0, 0.0, 0.0],
            [8.8, 0.0, 0.0],
            [8.8, 8.8, 0.0],
            [0.0, 8.8, 1.0]  # 1mm out of plane
        ])
        
        valid, msg = validate_tag_geometry(corners, tag_size_mm=8.8, tolerance_mm=0.2)
        
        assert not valid
        assert "Planarity error" in msg


class TestExtractionPipeline:
    """Test end-to-end extraction pipeline."""
    
    @pytest.fixture
    def mock_refpoints_file(self, tmp_path):
        """Create mock refpoints_U.json file."""
        refpoints_data = {
            "frame": "U",
            "units": "mm",
            "points": {
                # Tag 100 at origin
                "1000": [0.0, 0.0, 0.0],
                "1001": [8.8, 0.0, 0.0],
                "1002": [8.8, 8.8, 0.0],
                "1003": [0.0, 8.8, 0.0],
                # Tag 101 offset
                "1010": [20.0, 0.0, 0.0],
                "1011": [28.8, 0.0, 0.0],
                "1012": [28.8, 8.8, 0.0],
                "1013": [20.0, 8.8, 0.0]
            },
            "metadata": {
                "timestamp": "2026-01-14T10:00:00Z",
                "source": "test",
                "per_point_reprojection_errors_px": {
                    "1000": 0.4, "1001": 0.5, "1002": 0.6, "1003": 0.3,
                    "1010": 0.7, "1011": 0.8, "1012": 0.4, "1013": 0.5
                }
            }
        }
        
        refpoints_path = tmp_path / "refpoints_U.json"
        with open(refpoints_path, 'w') as f:
            json.dump(refpoints_data, f)
        
        return str(refpoints_path)
    
    def test_extract_two_implants(self, mock_refpoints_file, tmp_path):
        """Test extracting 2 implants from refpoints."""
        output_path = tmp_path / "implants_U.json"
        
        results = extract_implants_from_refpoints(
            mock_refpoints_file,
            str(output_path),
            tag_size_mm=8.8,
            validate_geometry=True
        )
        
        # Check results
        assert results["n_implants"] == 2
        assert results["tag_ids"] == [100, 101]
        
        # Load output
        with open(output_path, 'r') as f:
            implants_data = json.load(f)
        
        # Validate schema
        assert implants_data["frame"] == "U"
        assert implants_data["units"] == "mm"
        assert len(implants_data["implants"]) == 2
        
        # Check implant 1 (tag 100)
        implant1 = implants_data["implants"]["1"]
        assert implant1["marker_id"] == 100
        assert np.allclose(implant1["centroid_mm"], [4.4, 4.4, 0.0], atol=1e-6)
        assert np.allclose(np.abs(implant1["axis_vector"][2]), 1.0, atol=1e-6)
        
        # Check implant 2 (tag 101)
        implant2 = implants_data["implants"]["2"]
        assert implant2["marker_id"] == 101
        assert np.allclose(implant2["centroid_mm"], [24.4, 4.4, 0.0], atol=1e-6)
    
    def test_missing_corners(self, tmp_path):
        """Test handling of missing tag corners."""
        # Only 3 corners for tag 100
        refpoints_data = {
            "frame": "U",
            "units": "mm",
            "points": {
                "1000": [0.0, 0.0, 0.0],
                "1001": [8.8, 0.0, 0.0],
                "1002": [8.8, 8.8, 0.0]
                # Missing 1003
            },
            "metadata": {
                "timestamp": "2026-01-14T10:00:00Z",
                "source": "test"
            }
        }
        
        refpoints_path = tmp_path / "refpoints_incomplete.json"
        with open(refpoints_path, 'w') as f:
            json.dump(refpoints_data, f)
        
        output_path = tmp_path / "implants_U.json"
        
        # Should skip incomplete tag
        results = extract_implants_from_refpoints(
            str(refpoints_path),
            str(output_path),
            tag_size_mm=8.8
        )
        
        assert results["n_implants"] == 0
    
    def test_invalid_frame(self, tmp_path):
        """Test error on non-U-frame input."""
        refpoints_data = {
            "frame": "L",  # Wrong frame
            "units": "mm",
            "points": {}
        }
        
        refpoints_path = tmp_path / "refpoints_L.json"
        with open(refpoints_path, 'w') as f:
            json.dump(refpoints_data, f)
        
        output_path = tmp_path / "implants_U.json"
        
        with pytest.raises(ValueError, match="Expected U-frame"):
            extract_implants_from_refpoints(
                str(refpoints_path),
                str(output_path)
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
