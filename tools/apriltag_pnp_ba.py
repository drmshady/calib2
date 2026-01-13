"""
AprilTag Detection with PnP and Bundle Adjustment

Detects AprilTags, estimates camera poses using PnP, and refines with bundle adjustment.
Uses camera calibration from JSON file.

Usage:
    python tools/apriltag_pnp_ba.py <image_folder> <calibration.json> <layout.json> --output results.json
"""

import cv2
import numpy as np
import json
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from scipy.optimize import least_squares
from datetime import datetime
import argparse


def load_calibration(calib_path: str) -> Dict:
    """Load camera calibration from JSON file.
    
    Args:
        calib_path: Path to calibration JSON file
        
    Returns:
        Dict with K (3x3 intrinsic matrix) and D (distortion coefficients)
    """
    with open(calib_path, 'r') as f:
        data = json.load(f)
    
    K = np.array(data['intrinsics']['K'])
    D = np.array(data['distortion']['coefficients'])
    
    return {
        'K': K,
        'D': D,
        'image_size': (data['image_size']['width'], data['image_size']['height'])
    }


def load_tag_layout(layout_path: str) -> Dict:
    """Load AprilTag layout from JSON file.
    
    Args:
        layout_path: Path to layout JSON file
        
    Returns:
        Dict with tag_size_mm and 3D coordinates of tag corners
    """
    with open(layout_path, 'r') as f:
        data = json.load(f)
    
    tag_size = data['tag_size_mm']
    centers = data['centers_mm']
    
    # Generate 3D corner coordinates for each tag (Z=0, planar)
    # Corner order: TL, TR, BR, BL (OpenCV convention)
    half_size = tag_size / 2.0
    
    tag_corners_3d = {}
    for tag_id, center in centers.items():
        cx, cy = center
        corners = np.array([
            [cx - half_size, cy - half_size, 0.0],  # Top-left
            [cx + half_size, cy - half_size, 0.0],  # Top-right
            [cx + half_size, cy + half_size, 0.0],  # Bottom-right
            [cx - half_size, cy + half_size, 0.0]   # Bottom-left
        ], dtype=np.float32)
        tag_corners_3d[int(tag_id)] = corners
    
    return {
        'tag_size_mm': tag_size,
        'tag_corners_3d': tag_corners_3d
    }


def detect_apriltags(image: np.ndarray, dict_type: str = "DICT_4X4_1000") -> Tuple[List, List, List]:
    """Detect AprilTags in image.
    
    Args:
        image: Input image
        dict_type: ArUco dictionary type
        
    Returns:
        Tuple of (corners, ids, rejected)
    """
    # Dictionary mapping
    dict_mapping = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }
    
    dictionary = cv2.aruco.getPredefinedDictionary(dict_mapping[dict_type])
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    corners, ids, rejected = detector.detectMarkers(gray)
    
    return corners, ids, rejected


def estimate_pose_pnp(
    corners_2d: np.ndarray,
    corners_3d: np.ndarray,
    K: np.ndarray,
    D: np.ndarray
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
    """Estimate camera pose using PnP (Perspective-n-Point).
    
    Args:
        corners_2d: Detected 2D corners (Nx2)
        corners_3d: Known 3D corners (Nx3)
        K: Camera intrinsic matrix
        D: Distortion coefficients
        
    Returns:
        Tuple of (rvec, tvec, success)
    """
    if len(corners_2d) < 4:
        return None, None, False
    
    # Undistort points
    corners_2d_undist = cv2.undistortPoints(corners_2d.reshape(-1, 1, 2), K, D, P=K)
    corners_2d_undist = corners_2d_undist.reshape(-1, 2)
    
    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        corners_3d.reshape(-1, 3),
        corners_2d_undist.reshape(-1, 2),
        K,
        None,  # Already undistorted
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if success:
        # Refine with Levenberg-Marquardt
        rvec, tvec = cv2.solvePnPRefineLM(
            corners_3d.reshape(-1, 3),
            corners_2d_undist.reshape(-1, 2),
            K,
            None,
            rvec,
            tvec
        )
    
    return rvec, tvec, success


def project_points(points_3d: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """Project 3D points to 2D image coordinates.
    
    Args:
        points_3d: 3D points (Nx3)
        rvec: Rotation vector
        tvec: Translation vector
        K: Camera intrinsic matrix
        D: Distortion coefficients
        
    Returns:
        2D projected points (Nx2)
    """
    points_2d, _ = cv2.projectPoints(points_3d, rvec, tvec, K, D)
    return points_2d.reshape(-1, 2)


def compute_reprojection_error(
    corners_2d: np.ndarray,
    corners_3d: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    D: np.ndarray
) -> float:
    """Compute RMS reprojection error.
    
    Args:
        corners_2d: Detected 2D corners
        corners_3d: Known 3D corners
        rvec: Rotation vector
        tvec: Translation vector
        K: Camera intrinsic matrix
        D: Distortion coefficients
        
    Returns:
        RMS reprojection error in pixels
    """
    projected = project_points(corners_3d, rvec, tvec, K, D)
    errors = np.linalg.norm(corners_2d - projected, axis=1)
    return np.sqrt(np.mean(errors ** 2))


def bundle_adjustment_residuals(
    params: np.ndarray,
    n_cameras: int,
    n_points: int,
    camera_indices: np.ndarray,
    point_indices: np.ndarray,
    points_2d: np.ndarray,
    K: np.ndarray,
    D: np.ndarray
) -> np.ndarray:
    """Compute residuals for bundle adjustment.
    
    Args:
        params: Flattened parameters [camera_params (6*n_cameras), point_params (3*n_points)]
        n_cameras: Number of cameras
        n_points: Number of 3D points
        camera_indices: Camera index for each observation
        point_indices: Point index for each observation
        points_2d: Observed 2D points
        K: Camera intrinsic matrix
        D: Distortion coefficients
        
    Returns:
        Residuals (2 * n_observations)
    """
    camera_params = params[:n_cameras * 6].reshape((n_cameras, 6))
    points_3d = params[n_cameras * 6:].reshape((n_points, 3))
    
    residuals = []
    for i in range(len(camera_indices)):
        cam_idx = camera_indices[i]
        pt_idx = point_indices[i]
        
        rvec = camera_params[cam_idx, :3].reshape(3, 1)
        tvec = camera_params[cam_idx, 3:].reshape(3, 1)
        point_3d = points_3d[pt_idx].reshape(1, 3)
        
        projected = project_points(point_3d, rvec, tvec, K, D)
        residual = points_2d[i] - projected[0]
        residuals.extend(residual)
    
    return np.array(residuals)


def run_bundle_adjustment(
    initial_cameras: List[Dict],
    initial_points_3d: np.ndarray,
    observations: List[Dict],
    K: np.ndarray,
    D: np.ndarray,
    fix_points: bool = True
) -> Tuple[List[Dict], np.ndarray, Dict]:
    """Run bundle adjustment to refine camera poses and 3D points.
    
    Args:
        initial_cameras: List of dicts with 'rvec' and 'tvec'
        initial_points_3d: Initial 3D point positions (Nx3)
        observations: List of dicts with 'camera_idx', 'point_idx', 'point_2d'
        K: Camera intrinsic matrix
        D: Distortion coefficients
        fix_points: If True, only optimize camera poses (default for known layout)
        
    Returns:
        Tuple of (optimized_cameras, optimized_points_3d, info)
    """
    n_cameras = len(initial_cameras)
    n_points = len(initial_points_3d)
    
    # Build observation arrays
    camera_indices = np.array([obs['camera_idx'] for obs in observations])
    point_indices = np.array([obs['point_idx'] for obs in observations])
    points_2d = np.array([obs['point_2d'] for obs in observations])
    
    # Initial parameters
    camera_params = np.array([np.concatenate([cam['rvec'].flatten(), cam['tvec'].flatten()])
                              for cam in initial_cameras]).reshape(-1)
    point_params = initial_points_3d.flatten()
    params = np.concatenate([camera_params, point_params])
    
    print(f"\nBundle Adjustment:")
    print(f"  Cameras: {n_cameras}")
    print(f"  3D Points: {n_points}")
    print(f"  Observations: {len(observations)}")
    print(f"  Parameters: {len(params)} ({n_cameras * 6} camera + {n_points * 3} point)")
    
    # Compute initial error
    initial_residuals = bundle_adjustment_residuals(
        params, n_cameras, n_points, camera_indices, point_indices, points_2d, K, D
    )
    initial_error = np.sqrt(np.mean(initial_residuals ** 2))
    print(f"  Initial RMS error: {initial_error:.3f} px")
    
    # When fixing points, don't include them in optimization
    if fix_points:
        # Only optimize camera parameters
        camera_only_params = params[:n_cameras * 6]
        
        def residuals_camera_only(cam_params):
            full_params = np.concatenate([cam_params, point_params])
            return bundle_adjustment_residuals(
                full_params, n_cameras, n_points, camera_indices, point_indices, points_2d, K, D
            )
        
        result = least_squares(
            residuals_camera_only,
            camera_only_params,
            method='trf',
            verbose=0,
            ftol=1e-6,
            xtol=1e-6,
            max_nfev=100
        )
        
        # Reconstruct full parameters
        optimized_params = np.concatenate([result.x, point_params])
    else:
        # Optimize both camera and point parameters
        result = least_squares(
            bundle_adjustment_residuals,
            params,
            args=(n_cameras, n_points, camera_indices, point_indices, points_2d, K, D),
            method='trf',
            verbose=0,
            ftol=1e-6,
            xtol=1e-6,
            max_nfev=100
        )
        optimized_params = result.x
    
    # Extract results
    camera_params = optimized_params[:n_cameras * 6].reshape((n_cameras, 6))
    points_3d_opt = optimized_params[n_cameras * 6:].reshape((n_points, 3))
    
    optimized_cameras = []
    for i in range(n_cameras):
        optimized_cameras.append({
            'rvec': camera_params[i, :3].reshape(3, 1),
            'tvec': camera_params[i, 3:].reshape(3, 1)
        })
    
    # Final error
    final_residuals = result.fun
    final_error = np.sqrt(np.mean(final_residuals ** 2))
    
    print(f"  Final RMS error: {final_error:.3f} px")
    print(f"  Improvement: {initial_error - final_error:.3f} px ({(1 - final_error/initial_error)*100:.1f}%)")
    print(f"  Iterations: {result.nfev}")
    
    info = {
        'initial_error': float(initial_error),
        'final_error': float(final_error),
        'improvement_px': float(initial_error - final_error),
        'improvement_pct': float((1 - final_error/initial_error)*100),
        'iterations': int(result.nfev),
        'success': result.success
    }
    
    return optimized_cameras, points_3d_opt, info


def process_images(
    image_paths: List[str],
    calibration: Dict,
    layout: Dict,
    dict_type: str = 'DICT_4X4_1000',
    quality_gate_warning: float = 2.0,
    quality_gate_fail: float = 5.0
) -> Dict:
    """Process images: detect tags, estimate poses, run bundle adjustment.
    
    Args:
        image_paths: List of image file paths
        calib: Camera calibration dict
        layout: Tag layout dict
        dict_type: ArUco dictionary type
        quality_gate_warning: RMS threshold (px) for warnings
        quality_gate_fail: RMS threshold (px) for failures
        
    Returns:
        Results dict with poses, errors, and statistics
    """
    K = calibration['K']
    D = calibration['D']
    tag_corners_3d = layout['tag_corners_3d']
    
    results = {
        'images': [],
        'bundle_adjustment': None
    }
    
    initial_cameras = []
    observations = []
    point_3d_map = {}  # Map tag_id to point index
    all_points_3d = []
    
    print(f"\nProcessing {len(image_paths)} images...")
    
    for img_idx, img_path in enumerate(image_paths):
        img = cv2.imread(img_path)
        if img is None:
            print(f"  [{img_idx+1}/{len(image_paths)}] Failed to load: {img_path}")
            continue
        
        # Detect tags
        corners, ids, _ = detect_apriltags(img, dict_type)
        
        if ids is None or len(ids) == 0:
            print(f"  [{img_idx+1}/{len(image_paths)}] No tags detected: {Path(img_path).name}")
            continue
        
        # Show detected tags
        detected_ids = [int(tag_id[0]) for tag_id in ids]
        print(f"  [{img_idx+1}/{len(image_paths)}] Detected tags: {detected_ids} in {Path(img_path).name}")
        
        # Match detected tags with known layout
        detected_tags = {int(tag_id[0]): corners[i][0] for i, tag_id in enumerate(ids)}
        matched_2d = []
        matched_3d = []
        matched_tag_ids = []
        expected_tag_ids = list(tag_corners_3d.keys())
        
        for tag_id in detected_tags:
            if tag_id in tag_corners_3d:
                matched_2d.append(detected_tags[tag_id])
                matched_3d.append(tag_corners_3d[tag_id])
                matched_tag_ids.append(tag_id)
        
        if len(matched_2d) == 0:
            print(f"  [{img_idx+1}/{len(image_paths)}] No matching tags (expected {expected_tag_ids}, found {detected_ids}): {Path(img_path).name}")
            continue
        
        matched_2d = np.vstack(matched_2d)
        matched_3d = np.vstack(matched_3d)
        
        # Estimate pose with PnP
        rvec, tvec, success = estimate_pose_pnp(matched_2d, matched_3d, K, D)
        
        if not success:
            print(f"  [{img_idx+1}/{len(image_paths)}] PnP failed: {Path(img_path).name}")
            continue
        
        # Compute reprojection error
        rms_error = compute_reprojection_error(matched_2d, matched_3d, rvec, tvec, K, D)
        
        # Store results
        cam_data = {
            'image_path': str(img_path),
            'image_name': Path(img_path).name,
            'camera_idx': len(initial_cameras),
            'detected_tags': matched_tag_ids,
            'n_corners': len(matched_2d),
            'rvec': rvec.tolist(),
            'tvec': tvec.tolist(),
            'rms_error_px': float(rms_error)
        }
        
        results['images'].append(cam_data)
        initial_cameras.append({'rvec': rvec, 'tvec': tvec})
        
        # Build observations for bundle adjustment
        for corner_idx, (pt_2d, pt_3d) in enumerate(zip(matched_2d, matched_3d)):
            # Create unique point ID based on tag and corner
            tag_id = matched_tag_ids[corner_idx // 4]
            corner_in_tag = corner_idx % 4
            point_key = (tag_id, corner_in_tag)
            
            if point_key not in point_3d_map:
                point_3d_map[point_key] = len(all_points_3d)
                all_points_3d.append(pt_3d)
            
            point_idx = point_3d_map[point_key]
            
            observations.append({
                'camera_idx': len(initial_cameras) - 1,
                'point_idx': point_idx,
                'point_2d': pt_2d
            })
        
        print(f"  [{img_idx+1}/{len(image_paths)}] ✓ {len(matched_tag_ids)} tags, {len(matched_2d)} corners, RMS={rms_error:.3f} px: {Path(img_path).name}")
    
    print(f"\nSummary:")
    print(f"  Total images: {len(image_paths)}")
    print(f"  Images with poses: {len(initial_cameras)}")
    print(f"  Expected tag IDs: {sorted(tag_corners_3d.keys())}")
    
    if len(initial_cameras) < 2:
        print(f"\n✗ Insufficient images with detected tags: {len(initial_cameras)}")
        print(f"  Minimum required: 2")
        print(f"  Check that:")
        print(f"    1. Images contain AprilTags with IDs: {sorted(tag_corners_3d.keys())}")
        print(f"    2. Dictionary type is correct: {dict_type}")
        print(f"    3. Tags are visible and not blurred/occluded")
        return results
    
    # Run bundle adjustment
    all_points_3d = np.array(all_points_3d)
    optimized_cameras, optimized_points, ba_info = run_bundle_adjustment(
        initial_cameras, all_points_3d, observations, K, D, fix_points=True
    )
    
    # Update results with optimized poses
    for i, cam_data in enumerate(results['images']):
        cam_data['rvec_ba'] = optimized_cameras[i]['rvec'].tolist()
        cam_data['tvec_ba'] = optimized_cameras[i]['tvec'].tolist()
        
        # Recompute error with optimized pose
        img = cv2.imread(cam_data['image_path'])
        corners, ids, _ = detect_apriltags(img, dict_type)
        if ids is not None:
            detected_tags = {int(tag_id[0]): corners[i][0] for i, tag_id in enumerate(ids)}
            matched_2d = []
            matched_3d = []
            for tag_id in detected_tags:
                if tag_id in tag_corners_3d:
                    matched_2d.append(detected_tags[tag_id])
                    matched_3d.append(tag_corners_3d[tag_id])
            if len(matched_2d) > 0:
                matched_2d = np.vstack(matched_2d)
                matched_3d = np.vstack(matched_3d)
                rms_error_ba = compute_reprojection_error(
                    matched_2d, matched_3d,
                    optimized_cameras[i]['rvec'],
                    optimized_cameras[i]['tvec'],
                    K, D
                )
                cam_data['rms_error_ba_px'] = float(rms_error_ba)
    
    results['bundle_adjustment'] = ba_info
    
    # Compute statistics
    rms_errors = [img['rms_error_px'] for img in results['images']]
    rms_errors_ba = [img.get('rms_error_ba_px', img['rms_error_px']) for img in results['images']]
    
    results['statistics'] = {
        'n_images_processed': len(results['images']),
        'rms_error_mean_px': float(np.mean(rms_errors)),
        'rms_error_median_px': float(np.median(rms_errors)),
        'rms_error_max_px': float(np.max(rms_errors)),
        'rms_error_ba_mean_px': float(np.mean(rms_errors_ba)),
        'rms_error_ba_median_px': float(np.median(rms_errors_ba)),
        'rms_error_ba_max_px': float(np.max(rms_errors_ba))
    }
    
    # Quality gate assessment
    warnings = []
    failures = []
    for img in results['images']:
        rms = img.get('rms_error_ba_px', img['rms_error_px'])
        img_name = Path(img['image_path']).name
        if rms >= quality_gate_fail:
            failures.append({'image': img_name, 'rms_error': rms})
        elif rms >= quality_gate_warning:
            warnings.append({'image': img_name, 'rms_error': rms})
    
    results['quality_gate'] = {
        'warning_threshold_px': quality_gate_warning,
        'fail_threshold_px': quality_gate_fail,
        'warnings': warnings,
        'failures': failures,
        'n_warnings': len(warnings),
        'n_failures': len(failures),
        'pass_rate': (len(results['images']) - len(failures)) / len(results['images']) * 100 if results['images'] else 0
    }
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description="AprilTag detection with PnP and Bundle Adjustment"
    )
    parser.add_argument(
        "image_folder",
        help="Folder containing test images"
    )
    parser.add_argument(
        "calibration_json",
        help="Camera calibration JSON file"
    )
    parser.add_argument(
        "layout_json",
        help="Tag layout JSON file"
    )
    parser.add_argument(
        "--dict", type=str, default="DICT_4X4_1000",
        help="ArUco dictionary type (default: DICT_4X4_1000)"
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output JSON file for results"
    )
    parser.add_argument(
        "--quality-warning",
        type=float,
        default=2.0,
        help="RMS error threshold (px) for quality warning (default: 2.0)"
    )
    parser.add_argument(
        "--quality-fail",
        type=float,
        default=5.0,
        help="RMS error threshold (px) for quality failure (default: 5.0)"
    )
    
    args = parser.parse_args()
    
    # Load calibration
    print(f"Loading calibration from: {args.calibration_json}")
    calib = load_calibration(args.calibration_json)
    print(f"  Image size: {calib['image_size'][0]}×{calib['image_size'][1]} px")
    print(f"  fx={calib['K'][0,0]:.1f}, fy={calib['K'][1,1]:.1f}")
    print(f"  cx={calib['K'][0,2]:.1f}, cy={calib['K'][1,2]:.1f}")
    
    # Load layout
    print(f"\nLoading tag layout from: {args.layout_json}")
    layout = load_tag_layout(args.layout_json)
    print(f"  Tag size: {layout['tag_size_mm']} mm")
    print(f"  Number of tags: {len(layout['tag_corners_3d'])}")
    print(f"  Tag IDs: {sorted(layout['tag_corners_3d'].keys())}")
    
    # Find images
    image_folder = Path(args.image_folder)
    image_paths = []
    for ext in ['*.jpg', '*.jpeg', '*.png', '*.tif', '*.tiff', '*.JPG', '*.JPEG', '*.PNG', '*.TIF', '*.TIFF']:
        image_paths.extend(image_folder.glob(ext))
    image_paths = sorted(set(image_paths))
    
    if not image_paths:
        print(f"Error: No images found in {image_folder}")
        return 1
    
    print(f"\nFound {len(image_paths)} images in {image_folder}")
    
    # Process images
    results = process_images(
        [str(p) for p in image_paths],
        calib,
        layout,
        args.dict,
        quality_gate_warning=args.quality_warning,
        quality_gate_fail=args.quality_fail
    )
    
    # Print summary
    if results['images']:
        stats = results['statistics']
        print(f"\n{'='*80}")
        print(f"RESULTS SUMMARY")
        print(f"{'='*80}")
        print(f"Images processed: {stats['n_images_processed']}")
        print(f"\nPnP (initial):")
        print(f"  RMS error (mean):   {stats['rms_error_mean_px']:.3f} px")
        print(f"  RMS error (median): {stats['rms_error_median_px']:.3f} px")
        print(f"  RMS error (max):    {stats['rms_error_max_px']:.3f} px")
        
        if results['bundle_adjustment']:
            ba = results['bundle_adjustment']
            print(f"\nBundle Adjustment:")
            print(f"  RMS error (mean):   {stats['rms_error_ba_mean_px']:.3f} px")
            print(f"  RMS error (median): {stats['rms_error_ba_median_px']:.3f} px")
            print(f"  RMS error (max):    {stats['rms_error_ba_max_px']:.3f} px")
            print(f"  Improvement:        {ba['improvement_px']:.3f} px ({ba['improvement_pct']:.1f}%)")
            print(f"  Iterations:         {ba['iterations']}")
            print(f"  Status:             {'✓ Success' if ba['success'] else '✗ Failed'}")
        
        # Quality gate results
        if 'quality_gate' in results:
            qg = results['quality_gate']
            print(f"\nQuality Gate:")
            print(f"  Warning threshold:  {qg['warning_threshold_px']:.1f} px")
            print(f"  Fail threshold:     {qg['fail_threshold_px']:.1f} px")
            print(f"  Pass rate:          {qg['pass_rate']:.1f}% ({stats['n_images_processed'] - qg['n_failures']}/{stats['n_images_processed']})")
            
            if qg['n_failures'] > 0:
                print(f"  \n  ✗ FAILURES ({qg['n_failures']}):")
                for item in qg['failures']:
                    print(f"      {item['image']}: {item['rms_error']:.3f} px")
            
            if qg['n_warnings'] > 0:
                print(f"  \n  ⚠ WARNINGS ({qg['n_warnings']}):")
                for item in qg['warnings']:
                    print(f"      {item['image']}: {item['rms_error']:.3f} px")
            
            if qg['n_failures'] == 0 and qg['n_warnings'] == 0:
                print(f"  \n  ✓ All images passed quality gates")
    
    # Save results
    if args.output:
        output_data = {
            'timestamp': datetime.now().isoformat(),
            'calibration_file': args.calibration_json,
            'layout_file': args.layout_json,
            'dictionary': args.dict,
            'results': results
        }
        
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            json.dump(output_data, f, indent=2)
        print(f"\n✓ Results saved to: {output_path}")
    
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
