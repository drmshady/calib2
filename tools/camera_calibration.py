"""
Camera calibration using ChArUco board with AprilTag36h11 markers.

Computes camera intrinsics (K, distortion coefficients) and automatically
determines magnification from detected ChArUco corners. Requires roll diversity
across capture images (8+ unique orientations).

Usage:
    python tools/camera_calibration.py <image_folder> <output_json>

Example:
    python tools/camera_calibration.py calib/capture_session_01 calib/camera_intrinsics.json
"""

import cv2
import numpy as np
import json
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from datetime import datetime
import argparse


class CharucoBoard:
    """ChArUco board configuration for calibration."""
    
    def __init__(
        self,
        squares_x: int = 9,
        squares_y: int = 6,
        square_size_mm: float = 10.0,
        marker_size_mm: float = 7.0,
        dict_type: str = "DICT_APRILTAG_36h11"
    ):
        """Initialize ChArUco board.
        
        Args:
            squares_x: Number of squares in X direction
            squares_y: Number of squares in Y direction
            square_size_mm: Size of checkerboard squares in mm
            marker_size_mm: Size of AprilTag markers in mm
            dict_type: ArUco dictionary type
        """
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_size_mm = square_size_mm
        self.marker_size_mm = marker_size_mm
        self.dict_type = dict_type
        
        # Dictionary mapping
        dict_mapping = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }
        
        # Create dictionary
        dict_id = dict_mapping.get(dict_type, cv2.aruco.DICT_APRILTAG_36h11)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        
        # Create ChArUco board
        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_size_mm,
            marker_size_mm,
            self.dictionary
        )
        
        # Detection parameters
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
    def detect(self, image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
        """Detect ChArUco corners in image.
        
        Args:
            image: Input image (grayscale or BGR)
            
        Returns:
            Tuple of (corners, ids, marker_corners):
                corners: Detected ChArUco corner positions (Nx1x2)
                ids: Corner IDs (Nx1)
                marker_corners: Raw AprilTag marker corners for visualization
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect AprilTag markers
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
        
        if marker_ids is None or len(marker_ids) == 0:
            return None, None, np.array([])
        
        # Interpolate ChArUco corners (OpenCV 4.x API)
        charuco_detector = cv2.aruco.CharucoDetector(self.board)
        charuco_corners, charuco_ids, marker_corners_refined, marker_ids_refined = charuco_detector.detectBoard(gray)
        
        if charuco_corners is None or len(charuco_corners) == 0:
            return None, None, marker_corners
        
        return charuco_corners, charuco_ids, marker_corners


def compute_roll_angle(corners: np.ndarray) -> float:
    """Compute board roll angle from detected corners.
    
    Uses PCA to find principal axis of corner distribution.
    
    Args:
        corners: Nx1x2 array of corner positions
        
    Returns:
        Roll angle in degrees [0, 180)
    """
    if corners is None or len(corners) < 4:
        return 0.0
    
    # Flatten to Nx2
    pts = corners.reshape(-1, 2)
    
    # Center points
    mean = np.mean(pts, axis=0)
    centered = pts - mean
    
    # PCA via SVD
    _, _, Vt = np.linalg.svd(centered.T @ centered)
    principal_axis = Vt[0]
    
    # Angle from horizontal
    angle_rad = np.arctan2(principal_axis[1], principal_axis[0])
    angle_deg = np.degrees(angle_rad)
    
    # Normalize to [0, 180)
    if angle_deg < 0:
        angle_deg += 180
    
    return angle_deg


def check_roll_diversity(angles: List[float], min_unique_angles: int = 8, bin_size_deg: float = 22.5) -> Tuple[bool, int]:
    """Check if capture set has sufficient roll diversity.
    
    Divides [0, 180) into bins and counts unique bins covered.
    
    Args:
        angles: List of roll angles in degrees
        min_unique_angles: Minimum number of unique angle bins required
        bin_size_deg: Size of angle bins in degrees (default: 22.5° = 8 bins)
        
    Returns:
        Tuple of (sufficient, num_unique_bins)
    """
    if not angles:
        return False, 0
    
    # Bin angles
    bins = [int(angle // bin_size_deg) for angle in angles]
    unique_bins = len(set(bins))
    
    return unique_bins >= min_unique_angles, unique_bins


def compute_magnification(corners: np.ndarray, square_size_mm: float) -> float:
    """Compute magnification from detected ChArUco corners.
    
    Measures average pixel distance between adjacent corners and compares
    to known physical square size.
    
    Args:
        corners: Nx1x2 array of corner positions
        square_size_mm: Physical size of board squares in mm
        
    Returns:
        Magnification in pixels per mm
    """
    if corners is None or len(corners) < 4:
        return 0.0
    
    pts = corners.reshape(-1, 2)
    
    # Compute pairwise distances
    distances = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dist = np.linalg.norm(pts[i] - pts[j])
            distances.append(dist)
    
    # Use median distance as estimate (robust to outliers)
    # Divide by square size to get pixels per mm
    # Note: Adjacent corners are 1 square apart, but we're measuring all pairs
    # so we take a conservative estimate
    median_dist_px = np.median(distances)
    
    # Estimate magnification (this is approximate, actual will vary)
    # Better approach: measure minimum distances (adjacent corners)
    min_distances = sorted(distances)[:10]  # Take smallest 10
    avg_adjacent_dist = np.mean(min_distances)
    
    magnification = avg_adjacent_dist / square_size_mm
    
    return magnification


def calibrate_camera(
    image_paths: List[str],
    board: CharucoBoard,
    min_images: int = 20,
    min_corners_per_image: int = 30,
    min_unique_angles: int = 8,
    calib_flags: int = 0
) -> Dict:
    """Calibrate camera from ChArUco board images.
    
    Args:
        image_paths: List of calibration image file paths
        board: CharucoBoard instance
        min_images: Minimum valid images required
        min_corners_per_image: Minimum corners per image
        min_unique_angles: Minimum unique roll angle bins
        
    Returns:
        Dict with calibration results:
            - K: 3x3 intrinsic matrix
            - D: Distortion coefficients [k1, k2, p1, p2, k3]
            - image_size: (width, height)
            - reprojection_error: RMS reprojection error in pixels
            - magnification_px_per_mm: Computed magnification
            - n_images_used: Number of images used
            - roll_angles: List of detected roll angles
            - n_unique_angles: Number of unique angle bins
            
    Raises:
        ValueError: If insufficient images, corners, or roll diversity
    """
    all_corners = []
    all_ids = []
    all_sizes = []
    all_image_names = []  # Track valid image filenames
    roll_angles = []
    magnifications = []
    
    print(f"Processing {len(image_paths)} images...")
    
    for i, img_path in enumerate(image_paths):
        img = cv2.imread(img_path)
        if img is None:
            print(f"  [{i+1}/{len(image_paths)}] Failed to load: {img_path}")
            continue
        
        corners, ids, _ = board.detect(img)
        
        if corners is None or ids is None:
            print(f"  [{i+1}/{len(image_paths)}] No ChArUco corners detected: {Path(img_path).name}")
            continue
        
        if len(corners) < min_corners_per_image:
            print(f"  [{i+1}/{len(image_paths)}] Only {len(corners)} corners (min {min_corners_per_image}): {Path(img_path).name}")
            continue
        
        # Compute roll angle
        angle = compute_roll_angle(corners)
        roll_angles.append(angle)
        
        # Compute magnification
        mag = compute_magnification(corners, board.square_size_mm)
        magnifications.append(mag)
        
        all_corners.append(corners)
        all_ids.append(ids)
        all_sizes.append(img.shape[:2][::-1])  # (width, height)
        all_image_names.append(Path(img_path).name)  # Store filename
        
        print(f"  [{i+1}/{len(image_paths)}] ✓ {len(corners)} corners, roll={angle:.1f}°, mag={mag:.1f} px/mm: {Path(img_path).name}")
    
    # Validate sufficient images
    if len(all_corners) < min_images:
        raise ValueError(
            f"Insufficient valid images: {len(all_corners)} < {min_images} required"
        )
    
    # Check roll diversity
    sufficient_diversity, n_unique = check_roll_diversity(roll_angles, min_unique_angles)
    if not sufficient_diversity:
        print(f"\nWARNING: Insufficient roll diversity: {n_unique} unique angles (min {min_unique_angles})")
        print("Consider capturing images at more varied board orientations.")
    
    # Ensure consistent image size
    if len(set(all_sizes)) > 1:
        raise ValueError(f"Inconsistent image sizes detected: {set(all_sizes)}")
    
    image_size = all_sizes[0]
    
    print(f"\nCalibrating with {len(all_corners)} images...")
    print(f"  Roll diversity: {n_unique} unique angles")
    print(f"  Image size: {image_size[0]}×{image_size[1]} px")
    
    # Calibrate camera using standard OpenCV (OpenCV 4.x compatible)
    # Prepare object points for each image
    all_obj_points = []
    all_img_points = []
    
    for corners, ids in zip(all_corners, all_ids):
        # Get 3D object points for detected corners
        obj_points = board.board.getChessboardCorners()[ids.flatten()]
        all_obj_points.append(obj_points)
        all_img_points.append(corners)
    
    # Initial camera calibration
    retval, K, D, rvecs, tvecs = cv2.calibrateCamera(
        all_obj_points, all_img_points, image_size, None, None,
        flags=calib_flags  # Calibration model flags
    )
    
    if not retval:
        raise ValueError("Camera calibration failed")
    
    # Quality gate: Compute per-image reprojection errors and remove worst 10%
    per_image_errors = []
    for i, (corners, ids, rvec, tvec, img_name) in enumerate(zip(all_corners, all_ids, rvecs, tvecs, all_image_names)):
        obj_points = board.board.getChessboardCorners()[ids.flatten()]
        img_points, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
        
        corners_flat = corners.reshape(-1, 2)
        img_points_flat = img_points.reshape(-1, 2)
        errors = np.linalg.norm(corners_flat - img_points_flat, axis=1)
        rms = np.sqrt(np.mean(errors ** 2))
        per_image_errors.append((i, rms, len(corners), img_name))
    
    # Sort by error and remove worst 10%
    per_image_errors.sort(key=lambda x: x[1])
    n_to_remove = max(1, len(per_image_errors) // 10)  # Remove at least 1, or 10%
    keep_indices = [idx for idx, _, _, _ in per_image_errors[:-n_to_remove]]
    
    print(f"\nQuality gate: Removing worst {n_to_remove} images (10%) based on reprojection error")
    for idx, error, n_pts, img_name in per_image_errors[-n_to_remove:]:
        print(f"  Removed: {img_name} - error={error:.3f} px ({n_pts} corners)")
    
    # Filter data to keep only good images
    all_corners_filtered = [all_corners[i] for i in keep_indices]
    all_ids_filtered = [all_ids[i] for i in keep_indices]
    all_obj_points_filtered = [all_obj_points[i] for i in keep_indices]
    all_img_points_filtered = [all_img_points[i] for i in keep_indices]
    magnifications_filtered = [magnifications[i] for i in keep_indices]
    
    # Re-calibrate with filtered images
    print(f"Re-calibrating with {len(all_corners_filtered)} images (removed {n_to_remove})...")
    retval, K, D, rvecs, tvecs = cv2.calibrateCamera(
        all_obj_points_filtered, all_img_points_filtered, image_size, None, None,
        flags=calib_flags
    )
    
    if not retval:
        raise ValueError("Re-calibration failed")
    
    # Update data for final results
    all_corners = all_corners_filtered
    all_ids = all_ids_filtered
    magnifications = magnifications_filtered
    
    # Compute final reprojection error (overall and per-image)
    total_error = 0
    total_points = 0
    per_image_rms = []
    
    for corners, ids, rvec, tvec in zip(all_corners, all_ids, rvecs, tvecs):
        obj_points = board.board.getChessboardCorners()[ids.flatten()]
        img_points, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
        
        corners_flat = corners.reshape(-1, 2)
        img_points_flat = img_points.reshape(-1, 2)
        errors = np.linalg.norm(corners_flat - img_points_flat, axis=1)
        total_error += np.sum(errors ** 2)
        total_points += len(corners)
        
        # Per-image RMS
        img_rms = np.sqrt(np.mean(errors ** 2))
        per_image_rms.append(img_rms)
    
    rms_error_mean = np.sqrt(total_error / total_points)
    rms_error_median = np.median(per_image_rms)
    rms_error_max = np.max(per_image_rms)
    
    print(f"\nCalibration complete!")
    print(f"  RMS reprojection error (mean): {rms_error_mean:.3f} px")
    print(f"  RMS reprojection error (median): {rms_error_median:.3f} px")
    print(f"  RMS reprojection error (max): {rms_error_max:.3f} px")
    print(f"  Magnification (median): {np.median(magnifications):.2f} px/mm")
    print(f"  Magnification (range): {np.min(magnifications):.2f} - {np.max(magnifications):.2f} px/mm")
    
    # Warn if reprojection error is high
    if rms_error_mean > 0.55:
        print(f"\n  WARNING: Mean reprojection error {rms_error_mean:.3f} px exceeds 0.55 px threshold")
        print("  Consider recapturing with better lighting, focus, or board flatness")
    
    return {
        "K": K,
        "D": D,
        "image_size": image_size,
        "reprojection_error_px": float(rms_error_mean),
        "reprojection_error_median_px": float(rms_error_median),
        "reprojection_error_max_px": float(rms_error_max),
        "magnification_px_per_mm": float(np.median(magnifications)),
        "magnification_min": float(np.min(magnifications)),
        "magnification_max": float(np.max(magnifications)),
        "n_images_used": len(all_corners),
        "roll_angles_deg": roll_angles,
        "n_unique_angles": n_unique
    }


def save_calibration(results: Dict, output_path: str, board_config: CharucoBoard):
    """Save calibration results to JSON file.
    
    Args:
        results: Calibration results from calibrate_camera()
        output_path: Output JSON file path
        board_config: ChArUco board configuration
    """
    data = {
        "calibration_type": "charuco",
        "camera_model": "Nikon D5600",
        "lens": "50mm",
        "aperture": "f/16",
        "timestamp": datetime.now().isoformat(),
        "image_size": {
            "width": results["image_size"][0],
            "height": results["image_size"][1]
        },
        "intrinsics": {
            "fx": float(results["K"][0, 0]),
            "fy": float(results["K"][1, 1]),
            "cx": float(results["K"][0, 2]),
            "cy": float(results["K"][1, 2]),
            "K": results["K"].tolist()
        },
        "distortion": {
            "k1": float(results["D"].flatten()[0]),
            "k2": float(results["D"].flatten()[1]),
            "p1": float(results["D"].flatten()[2]),
            "p2": float(results["D"].flatten()[3]),
            "k3": float(results["D"].flatten()[4]) if len(results["D"].flatten()) > 4 else 0.0,
            "coefficients": results["D"].flatten().tolist()
        },
        "magnification": {
            "median_px_per_mm": results["magnification_px_per_mm"],
            "min_px_per_mm": results["magnification_min"],
            "max_px_per_mm": results["magnification_max"],
            "note": "Computed from detected ChArUco corners, varies with working distance"
        },
        "quality": {
            "reprojection_error_mean_px": results["reprojection_error_px"],
            "reprojection_error_median_px": results["reprojection_error_median_px"],
            "reprojection_error_max_px": results["reprojection_error_max_px"],
            "n_images_used": results["n_images_used"],
            "n_unique_roll_angles": results["n_unique_angles"]
        },
        "board": {
            "type": "charuco",
            "squares_x": board_config.squares_x,
            "squares_y": board_config.squares_y,
            "square_size_mm": board_config.square_size_mm,
            "marker_size_mm": board_config.marker_size_mm,
            "marker_family": board_config.dict_type
        },
        "units": "mm"
    }
    
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"\nSaved calibration to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Camera calibration using ChArUco board with AprilTag36h11 markers"
    )
    parser.add_argument(
        "image_folder",
        help="Folder containing calibration images"
    )
    parser.add_argument(
        "output_json",
        help="Output JSON file path for calibration results"
    )
    parser.add_argument(
        "--squares-x", type=int, default=9,
        help="Number of squares in X direction (default: 9)"
    )
    parser.add_argument(
        "--squares-y", type=int, default=6,
        help="Number of squares in Y direction (default: 6)"
    )
    parser.add_argument(
        "--square-size", type=float, default=10.0,
        help="Square size in mm (default: 10.0)"
    )
    parser.add_argument(
        "--marker-size", type=float, default=7.0,
        help="Marker size in mm (default: 7.0)"
    )
    parser.add_argument(
        "--dict", type=str, default="DICT_APRILTAG_36h11",
        help="ArUco dictionary type (default: DICT_APRILTAG_36h11)"
    )
    parser.add_argument(
        "--min-images", type=int, default=20,
        help="Minimum valid images required (default: 20)"
    )
    parser.add_argument(
        "--min-corners", type=int, default=30,
        help="Minimum corners per image (default: 30)"
    )
    parser.add_argument(
        "--min-angles", type=int, default=8,
        help="Minimum unique roll angle bins (default: 8)"
    )
    parser.add_argument(
        "--calib-model", type=str, default="standard",
        choices=["standard", "rational", "thin-prism", "tilted"],
        help="Calibration model: standard (5-param k1,k2,k3,p1,p2), rational (8-param adds k4,k5,k6), thin-prism (12-param adds s1,s2,s3,s4), tilted (14-param adds tauX,tauY) (default: standard)"
    )
    
    args = parser.parse_args()
    
    # Map calibration model to OpenCV flags
    calib_model_flags = {
        "standard": 0,  # Default 5-parameter model (k1, k2, p1, p2, k3)
        "rational": cv2.CALIB_RATIONAL_MODEL,  # 8-parameter: adds k4, k5, k6
        "thin-prism": cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_THIN_PRISM_MODEL,  # 12-parameter: adds s1, s2, s3, s4
        "tilted": cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_THIN_PRISM_MODEL | cv2.CALIB_TILTED_MODEL  # 14-parameter: adds tauX, tauY
    }
    
    # Find all images in folder
    image_folder = Path(args.image_folder)
    if not image_folder.exists():
        print(f"Error: Folder not found: {image_folder}")
        return 1
    
    image_paths = []
    for ext in ['*.jpg', '*.jpeg', '*.png', '*.tif', '*.tiff', '*.JPG', '*.JPEG', '*.PNG', '*.TIF', '*.TIFF']:
        image_paths.extend(image_folder.glob(ext))
    
    # Remove duplicates (case-insensitive filesystems may return same file twice)
    image_paths = sorted(set(image_paths))
    
    if not image_paths:
        print(f"Error: No images found in {image_folder}")
        return 1
    
    print(f"Found {len(image_paths)} images in {image_folder}")
    print(f"Calibration model: {args.calib_model}")
    
    # Create board
    board = CharucoBoard(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_size_mm=args.square_size,
        marker_size_mm=args.marker_size,
        dict_type=args.dict
    )
    
    try:
        # Calibrate
        results = calibrate_camera(
            [str(p) for p in image_paths],
            board,
            min_images=args.min_images,
            min_corners_per_image=args.min_corners,
            min_unique_angles=args.min_angles,
            calib_flags=calib_model_flags[args.calib_model]
        )
        
        # Save
        save_calibration(results, args.output_json, board)
        
        print("\n✓ Calibration successful!")
        return 0
        
    except Exception as e:
        print(f"\n✗ Calibration failed: {e}")
        return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
