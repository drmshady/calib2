"""
Calibration loader and utilities for camera intrinsics.

Provides functions to load camera calibration data, undistort points,
and validate calibration quality.

COORDINATE CONVENTION:
        - Distorted detections start as pixel coordinates.
        - Project standard for reconstruction/bundle adjustment is **undistorted pixels**
            (use `cv2.undistortPoints(..., P=K)`), because residual thresholds are in pixels.
        - For epipolar/essential-matrix workflows, **normalized** coordinates are sometimes
            required; this module supports both.

Functions:
    load_calibration: Load K, D, image_size from JSON
    undistort_points: Convert distorted pixels to undistorted pixels (default) or normalized
    get_magnification: Extract magnification from calibration metadata
    CalibrationQA: Validation class for calibration quality checks
"""

import numpy as np
import json
import cv2
from pathlib import Path
from typing import Tuple, Dict, Optional, Literal


def load_calibration(filepath: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int], Dict]:
    """Load camera calibration from JSON file.
    
    Args:
        filepath: Path to calibration JSON (e.g., camera_intrinsics.json)
        
    Returns:
        Tuple of (K, D, image_size, metadata):
            K: 3x3 camera intrinsic matrix
            D: Distortion coefficients [k1, k2, p1, p2, k3]
            image_size: (width, height) in pixels
            metadata: Dict with magnification, quality metrics, etc.
            
    Raises:
        FileNotFoundError: If calibration file doesn't exist
        ValueError: If calibration format is invalid
    """
    if not Path(filepath).exists():
        raise FileNotFoundError(f"Calibration file not found: {filepath}")
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    # Validate required fields
    required_fields = ["intrinsics", "distortion", "image_size"]
    for field in required_fields:
        if field not in data:
            raise ValueError(f"Missing required field: {field}")
    
    # Extract intrinsics
    if "K" in data["intrinsics"]:
        K = np.array(data["intrinsics"]["K"], dtype=np.float64)
    else:
        # Build from fx, fy, cx, cy
        fx = data["intrinsics"]["fx"]
        fy = data["intrinsics"]["fy"]
        cx = data["intrinsics"]["cx"]
        cy = data["intrinsics"]["cy"]
        K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
    
    # Extract distortion coefficients
    if "coefficients" in data["distortion"]:
        D = np.array(data["distortion"]["coefficients"], dtype=np.float64)
    else:
        # Build from individual coefficients
        D = np.array([
            data["distortion"]["k1"],
            data["distortion"]["k2"],
            data["distortion"]["p1"],
            data["distortion"]["p2"],
            data["distortion"].get("k3", 0.0)
        ], dtype=np.float64)
    
    # Extract image size
    image_size = (
        data["image_size"]["width"],
        data["image_size"]["height"]
    )
    
    # Extract metadata
    metadata = {
        "calibration_type": data.get("calibration_type", "unknown"),
        "camera_model": data.get("camera_model", "unknown"),
        "lens": data.get("lens", "unknown"),
        "aperture": data.get("aperture", "unknown"),
        "timestamp": data.get("timestamp", "unknown"),
        "reprojection_error_px": data.get("quality", {}).get("reprojection_error_px", None),
        "magnification_px_per_mm": data.get("magnification", {}).get("median_px_per_mm", None),
        "n_images_used": data.get("quality", {}).get("n_images_used", None)
    }
    
    return K, D, image_size, metadata


def undistort_points(
    points: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    output: Literal["pixels", "normalized"] = "pixels",
) -> np.ndarray:
    """Undistort distorted pixel coordinates.

    Removes lens distortion from 2D image points.

    Output modes:
        - output='pixels' (DEFAULT): undistorted pixel coordinates (project standard)
          Equivalent to `cv2.undistortPoints(..., P=K)`.
        - output='normalized': normalized (dimensionless) camera coordinates
          Equivalent to `cv2.undistortPoints(..., P=None)`.
    
    Args:
        points: Nx2 array of distorted pixel coordinates
        K: 3x3 camera intrinsic matrix
        D: Distortion coefficients [k1, k2, p1, p2, k3]
        
    Returns:
        Nx2 array of undistorted coordinates (pixels or normalized per `output`)
    """
    points = np.array(points, dtype=np.float64)
    if points.ndim == 1:
        points = points.reshape(1, -1)
    
    # Reshape for OpenCV (Nx1x2)
    points_cv = points.reshape(-1, 1, 2)
    
    if output not in ("pixels", "normalized"):
        raise ValueError(f"output must be 'pixels' or 'normalized', got: {output!r}")

    P = K if output == "pixels" else None
    points_undistorted = cv2.undistortPoints(points_cv, K, D, P=P)
    
    # Reshape back to Nx2
    return points_undistorted.reshape(-1, 2)


def redistort_points(
    points_normalized: np.ndarray,
    K: np.ndarray,
    D: np.ndarray
) -> np.ndarray:
    """Re-apply distortion to normalized coordinates (inverse of undistort_points).
    
    Converts normalized camera coordinates back to distorted pixel coordinates.
    Used for validation and round-trip testing.
    
    Args:
        points_normalized: Nx2 array of normalized undistorted coordinates
        K: 3x3 camera intrinsic matrix
        D: Distortion coefficients [k1, k2, p1, p2, k3]
        
    Returns:
        Nx2 array of distorted pixel coordinates
    """
    points = np.array(points_normalized, dtype=np.float64)
    if points.ndim == 1:
        points = points.reshape(1, -1)
    
    # Add Z=1 to make homogeneous (Nx3)
    points_3d = np.hstack([points, np.ones((len(points), 1))])
    
    # Project using OpenCV projectPoints
    # Need dummy rvec/tvec (identity transform)
    rvec = np.zeros(3, dtype=np.float64)
    tvec = np.zeros(3, dtype=np.float64)
    
    points_distorted, _ = cv2.projectPoints(points_3d, rvec, tvec, K, D)
    
    return points_distorted.reshape(-1, 2)


def get_magnification(filepath: str) -> Optional[float]:
    """Extract magnification from calibration metadata.
    
    Args:
        filepath: Path to calibration JSON
        
    Returns:
        Magnification in pixels per mm, or None if not available
    """
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    return data.get("magnification", {}).get("median_px_per_mm", None)


class CalibrationQA:
    """Calibration quality assurance checker.
    
    Validates calibration quality via pre-flight checks:
    - Reprojection error within acceptable range
    - Principal point near image center
    - Aspect ratio near 1.0
    - Distortion magnitude reasonable
    - Round-trip undistort→redistort accuracy
    """
    
    def __init__(
        self,
        K: np.ndarray,
        D: np.ndarray,
        image_size: Tuple[int, int],
        metadata: Dict
    ):
        """Initialize QA checker.
        
        Args:
            K: 3x3 intrinsic matrix
            D: Distortion coefficients
            image_size: (width, height) in pixels
            metadata: Calibration metadata dict
        """
        self.K = K
        self.D = D
        self.image_size = image_size
        self.metadata = metadata
        
        self.warnings = []
        self.errors = []
    
    def check_reprojection_error(self, max_error_px: float = 0.5):
        """Check reprojection error is within acceptable range."""
        reproj_error = self.metadata.get("reprojection_error_px")
        
        if reproj_error is None:
            self.warnings.append("Reprojection error not available in metadata")
            return
        
        if reproj_error > max_error_px:
            self.errors.append(
                f"Reprojection error {reproj_error:.3f} px exceeds threshold {max_error_px:.3f} px"
            )
        else:
            print(f"  ✓ Reprojection error: {reproj_error:.3f} px")
    
    def check_principal_point(self, max_offset_px: float = 100.0):
        """Check principal point is near image center."""
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        
        img_center_x = self.image_size[0] / 2
        img_center_y = self.image_size[1] / 2
        
        offset_x = abs(cx - img_center_x)
        offset_y = abs(cy - img_center_y)
        
        if offset_x > max_offset_px or offset_y > max_offset_px:
            self.warnings.append(
                f"Principal point offset from center: ({offset_x:.1f}, {offset_y:.1f}) px "
                f"(max {max_offset_px:.1f} px)"
            )
        else:
            print(f"  ✓ Principal point near center: ({cx:.1f}, {cy:.1f})")
    
    def check_aspect_ratio(self, max_deviation: float = 0.01):
        """Check pixel aspect ratio is near 1.0."""
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        
        aspect_ratio = fx / fy
        deviation = abs(aspect_ratio - 1.0)
        
        if deviation > max_deviation:
            self.warnings.append(
                f"Aspect ratio {aspect_ratio:.4f} deviates from 1.0 by {deviation:.4f}"
            )
        else:
            print(f"  ✓ Aspect ratio: {aspect_ratio:.4f}")
    
    def check_distortion_magnitude(self):
        """Check distortion coefficients are reasonable."""
        k1 = abs(self.D[0])
        k2 = abs(self.D[1])
        
        # Typical values for DSLR lenses: k1 in [-0.5, 0.5], k2 in [-0.3, 0.3]
        if k1 > 0.5:
            self.warnings.append(f"Large k1 distortion: {self.D[0]:.4f}")
        
        if k2 > 0.3:
            self.warnings.append(f"Large k2 distortion: {self.D[1]:.4f}")
        
        print(f"  ✓ Distortion coefficients: k1={self.D[0]:.4f}, k2={self.D[1]:.4f}")
    
    def check_round_trip_accuracy(self, n_test_points: int = 100, max_error_px: float = 1e-3):
        """Check undistort→redistort round-trip accuracy."""
        # Generate random test points across image
        rng = np.random.RandomState(42)
        test_points = rng.uniform(
            low=[0, 0],
            high=[self.image_size[0], self.image_size[1]],
            size=(n_test_points, 2)
        )
        
        # Round-trip: distorted → normalized → distorted
        points_normalized = undistort_points(test_points, self.K, self.D, output="normalized")
        points_recovered = redistort_points(points_normalized, self.K, self.D)
        
        # Compute error
        errors = np.linalg.norm(test_points - points_recovered, axis=1)
        max_round_trip_error = np.max(errors)
        mean_round_trip_error = np.mean(errors)
        
        if max_round_trip_error > max_error_px:
            self.errors.append(
                f"Round-trip error {max_round_trip_error:.2e} px exceeds {max_error_px:.2e} px"
            )
        else:
            print(f"  ✓ Round-trip accuracy: {mean_round_trip_error:.2e} px (max {max_round_trip_error:.2e} px)")
    
    def run_all_checks(self) -> bool:
        """Run all QA checks.
        
        Returns:
            True if all checks pass (no errors), False otherwise
        """
        print("\nCalibration Quality Checks:")
        
        self.check_reprojection_error()
        self.check_principal_point()
        self.check_aspect_ratio()
        self.check_distortion_magnitude()
        self.check_round_trip_accuracy()
        
        # Print warnings
        if self.warnings:
            print("\nWarnings:")
            for warning in self.warnings:
                print(f"  ⚠ {warning}")
        
        # Print errors
        if self.errors:
            print("\nErrors:")
            for error in self.errors:
                print(f"  ✗ {error}")
            return False
        
        print("\n✓ All calibration QA checks passed")
        return True


def validate_calibration_file(filepath: str) -> bool:
    """Run full validation on calibration file.
    
    Args:
        filepath: Path to calibration JSON
        
    Returns:
        True if validation passes, False otherwise
    """
    try:
        K, D, image_size, metadata = load_calibration(filepath)
        
        qa = CalibrationQA(K, D, image_size, metadata)
        return qa.run_all_checks()
        
    except Exception as e:
        print(f"Validation failed: {e}")
        return False


# CLI interface for validation
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python calibration_loader.py <calibration.json>")
        sys.exit(1)
    
    filepath = sys.argv[1]
    
    print(f"Validating: {filepath}")
    success = validate_calibration_file(filepath)
    
    sys.exit(0 if success else 1)
